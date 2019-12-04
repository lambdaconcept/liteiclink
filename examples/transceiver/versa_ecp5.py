#!/usr/bin/env python3

# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from nmigen import *
from nmigen.lib.cdc import ResetSynchronizer

from nmigen.build import *
from nmigen.vendor.lattice_ecp5 import *
from nmigen_boards.resources import *

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

        Resource("pcie_tx", 0, DiffPairs("W4", "W5", dir="o")), # , Attrs(IO_TYPE="LVDS")),
        Resource("pcie_rx", 0, DiffPairs("Y5", "Y6", dir="i")), # , Attrs(IO_TYPE="LVDS")),
    ]
    connectors  = []


# CRG ----------------------------------------------------------------------------------------------

class _CRG(Elaboratable):
    def __init__(self, platform, sys_clk_freq, refclk_freq):
        self.sys_clk_freq = sys_clk_freq
        self.refclk_freq = refclk_freq

    def elaborate(self, platform):
        m = Module()

        m.domains.sync = cd_sync = ClockDomain()
        m.domains.ref = cd_ref = ClockDomain()

        # # #

        # self.cd_sys.clk.attr.add("keep")

        # clk / rst
        clk100 = platform.request("clk100")
        rst = platform.request("rst")
        # platform.add_period_constraint(clk100, 1e9/100e6) # XXX PO

        # pll
        m.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100.i, 100e6)
        pll.create_clkout(cd_sync, self.sys_clk_freq)
        pll.create_clkout(cd_ref, self.refclk_freq)
        m.submodules += ResetSynchronizer(rst.i, domain="sync")
        m.submodules += ResetSynchronizer(rst.i, domain="ref")

        return m


# SerDesTestSoC ------------------------------------------------------------------------------------

class SerDesTestTop(Elaboratable):
    def __init__(self, connector="pcie"):
        assert connector in ["sma", "pcie"]
        self.connector = connector

    def elaborate(self, platform):
        m = Module()

        #   # CRG --------------------------------------------------------------------------------------
        sys_clk_freq = int(100e6)
        refclk_freq = int(200e6)
        m.submodules.crg = _CRG(platform, sys_clk_freq, refclk_freq)

        #   # SerDes RefClk ----------------------------------------------------------------------------
        #   # refclk_pads = platform.request("refclk", 1)
        #   # self.comb += platform.request("refclk_en").eq(1)
        #   # self.comb += platform.request("refclk_rst_n").eq(1)
        #   # refclk = Signal()
        #   # self.specials.extref0 = Instance("EXTREFB",
        #   #     i_REFCLKP=refclk_pads.p,
        #   #     i_REFCLKN=refclk_pads.n,
        #   #     o_REFCLKO=refclk,
        #   #     p_REFCK_PWDNB="0b1",
        #   #     p_REFCK_RTERM="0b1", # 100 Ohm
        #   # )
        #   # self.extref0.attr.add(("LOC", "EXTREF0"))

        # SerDes PLL -------------------------------------------------------------------------------
        # serdes_pll = SerDesECP5PLL(refclk, refclk_freq=156.25e6, linerate=1.25e9)
        serdes_pll = SerDesECP5PLL(ClockSignal("ref"), refclk_freq=refclk_freq, linerate=5e9)
        m.submodules += serdes_pll

        # SerDes -----------------------------------------------------------------------------------
        tx_pads = platform.request(self.connector + "_tx", 0, dir="-")
        rx_pads = platform.request(self.connector + "_rx", 0, dir="-")
        channel = 1 if self.connector == "sma" else 0
        serdes  = SerDesECP5(serdes_pll, tx_pads, rx_pads, channel=channel, data_width=20, clock_aligner=True)
        serdes.add_stream_endpoints()
        m.submodules += serdes
        # platform.add_period_constraint(serdes.txoutclk, 1e9/serdes.tx_clk_freq)
        # platform.add_period_constraint(serdes.rxoutclk, 1e9/serdes.rx_clk_freq)

        # Test -------------------------------------------------------------------------------------
        counter = Signal(32)
        m.d.tx += counter.eq(counter + 1)

        # K28.5 and slow counter --> TX
        m.d.comb += [
            serdes.sink.valid.eq(1),
            serdes.sink.ctrl.eq(0b01),
            serdes.sink.data[0:8].eq((5 << 5) | 28),
            serdes.sink.data[8:16].eq(counter[26:]),
        ]

        # RX (slow counter) --> Leds
        m.d.rx += [
            serdes.rx_align.eq(1),
            serdes.source.ready.eq(1),
            platform.request("led", 4).eq(serdes.source.data[ 8]),
            platform.request("led", 5).eq(serdes.source.data[ 9]),
            platform.request("led", 6).eq(serdes.source.data[10]),
            platform.request("led", 7).eq(serdes.source.data[11]),
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

# Build --------------------------------------------------------------------------------------------

def main():
    platform = Platform()
    platform.build(SerDesTestTop(), name="versa_ecp5")

if __name__ == "__main__":
    main()
