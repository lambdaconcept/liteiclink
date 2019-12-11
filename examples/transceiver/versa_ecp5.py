#!/usr/bin/env python3

# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import sys

from nmigen import *
from nmigen.lib.cdc import ResetSynchronizer

from nmigen.build import *
from nmigen.vendor.lattice_ecp5 import *
from nmigen_boards.resources import *

from lib.soc.cores.rs232 import *
from lib.soc.interconnect import stream

from porting.litex.soc.cores.clock import ECP5PLL
from liteiclink.transceiver.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

class Platform(LatticeECP5Platform):
    device      = "LFE5UM5G-45F"
    package     = "BG381"
    speed       = "8"
    default_clk = "clk100"
    default_rst = "rst"
    resources   = [
        Resource("rst", 0, PinsN("T1", dir="i"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("clk100", 0, DiffPairs("P3", "P4", dir="i"),
                 Clock(100e6), Attrs(IO_TYPE="LVDS")),

        *LEDResources(pins="E16 D17 D18 E18 F17 F18 E17 F16",
                      attrs=Attrs(IO_TYPE="LVCMOS25")),

        UARTResource(0,
            rx="C11", tx="A11",
            attrs=Attrs(IO_TYPE="LVCMOS33", PULLMODE="UP")
        ),

        Resource("pcie_tx", 0, DiffPairs("W4", "W5", dir="o")),
        Resource("pcie_rx", 0, DiffPairs("Y5", "Y6", dir="i")),
    ]
    connectors  = []


# CRG ----------------------------------------------------------------------------------------------

class _CRG(Elaboratable):
    def __init__(self, platform, sys_clk_freq, refclk_from_pll, refclk_freq):
        self.sys_clk_freq = sys_clk_freq
        self.refclk_from_pll = refclk_from_pll
        self.refclk_freq = refclk_freq

    def elaborate(self, platform):
        m = Module()

        m.domains.sync = cd_sync = ClockDomain()
        m.domains.ref = cd_ref = ClockDomain(reset_less=True)
        m.domains.por = cd_por = ClockDomain(reset_less=True)

        # # #

        # self.cd_sys.clk.attr.add("keep")

        # clk / rst
        clk100 = platform.request("clk100")
        rst = platform.request("rst")
        # platform.add_period_constraint(clk100, 1e9/100e6)

        # power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done = Signal()
        m.d.comb += cd_por.clk.eq(ClockSignal())
        m.d.comb += por_done.eq(por_count == 0)
        with m.If(~por_done):
            m.d.por += por_count.eq(por_count - 1)

        # pll
        m.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100.i, 100e6)
        pll.create_clkout(cd_sync, self.sys_clk_freq)
        if self.refclk_from_pll:
            pll.create_clkout(cd_ref, self.refclk_freq)
        m.submodules += ResetSynchronizer(~por_done | ~pll.locked | rst.i, domain="sync")

        return m


# SerDesTestSoC ------------------------------------------------------------------------------------

class SerDesTestTop(Elaboratable):
    def __init__(self, connector="pcie", linerate=2.5e9):
        assert linerate in [2.5e9, 5e9]
        assert connector in ["sma", "pcie"]
        self.connector = connector
        self.linerate = linerate

    def elaborate(self, platform):
        m = Module()

        # CRG --------------------------------------------------------------------------------------
        sys_clk_freq = int(100e6)
        if self.linerate == 2.5e9:
            refclk_from_pll = False
            refclk_freq     = 156.25e6
        else:
            refclk_from_pll = True
            refclk_freq     = 200e6
        m.submodules.crg = crg = _CRG(platform, sys_clk_freq, refclk_from_pll, refclk_freq)

        # SerDes RefClk ----------------------------------------------------------------------------
        if refclk_from_pll:
            refclk = ClockSignal("ref")
        else:
            refclk_pads = platform.request("refclk", 1)
            m.d.comb += platform.request("refclk_en").eq(1)
            m.d.comb += platform.request("refclk_rst_n").eq(1)
            refclk = Signal()
            self.specials.extref0 = Instance("EXTREFB",
                i_REFCLKP=refclk_pads.p,
                i_REFCLKN=refclk_pads.n,
                o_REFCLKO=refclk,
                p_REFCK_PWDNB="0b1",
                p_REFCK_RTERM="0b1", # 100 Ohm
            )
            self.extref0.attr.add(("LOC", "EXTREF0"))

        # SerDes PLL -------------------------------------------------------------------------------
        serdes_pll = SerDesECP5PLL(ClockSignal("ref"), refclk_freq=refclk_freq, linerate=self.linerate)
        m.submodules += serdes_pll

        # SerDes -----------------------------------------------------------------------------------
        tx_pads = platform.request(self.connector + "_tx", 0, dir="-")
        rx_pads = platform.request(self.connector + "_rx", 0, dir="-")
        channel = 1 if self.connector == "sma" else 0
        serdes  = SerDesECP5(serdes_pll, tx_pads, rx_pads,
            channel       = channel,
            data_width    = 20)
        serdes.add_stream_endpoints()
        m.submodules += serdes
        # platform.add_period_constraint(serdes.txoutclk, 1e9/serdes.tx_clk_freq)
        # platform.add_period_constraint(serdes.rxoutclk, 1e9/serdes.rx_clk_freq)

        # Uart -------------------------------------------------------------------------------------
        uart = RS232PHY(platform.request("uart"), sys_clk_freq, baudrate=115200)
        m.submodules += uart

        # CDCtx ------------------------------------------------------------------------------------
        cdctx = stream.AsyncFIFO([("data", 8)], 8, r_domain="tx", w_domain="sync")
        m.submodules += cdctx
        m.d.comb += uart.source.connect(cdctx.sink)

        # CDCrx ------------------------------------------------------------------------------------
        cdcrx = stream.AsyncFIFO([("data", 8)], 8, r_domain="sync", w_domain="rx")
        m.submodules += cdcrx
        m.d.comb += cdcrx.source.connect(uart.sink)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        m.d.tx += counter.eq(counter + 1)

        # TX path ----------------------------------------------------------------------------------
        with m.If(cdctx.source.valid):
            m.d.comb += [
                cdctx.source.ready.eq(1),
                serdes.sink.valid.eq(1), # serdes.sink always ready
                serdes.sink.ctrl.eq(0b00),
                serdes.sink.data[0:8].eq(0),
                serdes.sink.data[8:16].eq(cdctx.source.data),
            ]
        with m.Else():
            # K28.5 and slow counter --> TX
            m.d.comb += [
                serdes.sink.valid.eq(1),
                serdes.sink.ctrl.eq(0b01),
                serdes.sink.data[0:8].eq((5 << 5) | 28),
                serdes.sink.data[8:16].eq(counter[26:]),
            ]

        # RX path ----------------------------------------------------------------------------------
        m.d.rx += [
            serdes.rx_align.eq(1),
            serdes.source.ready.eq(1),
        ]

        counter = Signal(8)
        with m.If(serdes.source.ctrl == 0): # serdes.source always valid
            # No word aligner, so look for 0 and redirect the other byte to the uart
            with m.If(serdes.source.data[0:8] == 0):
                m.d.rx += counter.eq(serdes.source.data[8:])
            with m.Else():
                m.d.rx += counter.eq(serdes.source.data[0:])
            m.d.rx += [
                cdcrx.sink.valid.eq(1), # assume fifo ready
                cdcrx.sink.data.eq(counter),
            ]
        with m.Else():
            # RX (slow counter) --> Leds
            # No word aligner, so look for K28.5 and redirect the other byte to the leds
            with m.If(serdes.source.data[0:8] == ((5 << 5) | 28)):
                m.d.rx += counter.eq(serdes.source.data[8:])
            with m.Else():
                m.d.rx += counter.eq(serdes.source.data[0:])
            m.d.rx += [
                cdcrx.sink.valid.eq(0),
                platform.request("led", 4).eq(~counter[0]),
                platform.request("led", 5).eq(~counter[1]),
                platform.request("led", 6).eq(~counter[2]),
                platform.request("led", 7).eq(~counter[3]),
            ]

        # Leds -------------------------------------------------------------------------------------
        sys_counter = Signal(32)
        m.d.sync += sys_counter.eq(sys_counter + 1)
        m.d.comb += platform.request("led", 0).eq(sys_counter[26])

        rx_counter = Signal(32)
        m.d.rx += rx_counter.eq(rx_counter + 1)
        m.d.comb += platform.request("led", 1).eq(rx_counter[26])

        tx_counter = Signal(32)
        m.d.tx += tx_counter.eq(rx_counter + 1)
        m.d.comb += platform.request("led", 2).eq(tx_counter[26])

        return m

# Load ---------------------------------------------------------------------------------------------
def load():
    import os
    f = open("ecp5-versa5g.cfg", "w")
    f.write(
"""
interface ftdi
ftdi_vid_pid 0x0403 0x6010
ftdi_channel 0
ftdi_layout_init 0xfff8 0xfffb
reset_config none
adapter_khz 25000
jtag newtap ecp5 tap -irlen 8 -expected-id 0x81112043
""")
    f.close()
    os.system("openocd -f ecp5-versa5g.cfg -c \"transport select jtag; init; svf build/versa_ecp5.svf; exit\"")
    exit()

# Build --------------------------------------------------------------------------------------------

def main():
    if "load" in sys.argv[1:]:
        load()
    platform = Platform()
    platform.build(SerDesTestTop(linerate=5e9), name="versa_ecp5")

if __name__ == "__main__":
    main()
