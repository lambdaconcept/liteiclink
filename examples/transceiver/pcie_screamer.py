#!/usr/bin/env python3

from nmigen import *
from nmigen.lib.cdc import ResetSynchronizer

from nmigen.build import *
from nmigen.vendor.xilinx_7series import *
from nmigen_boards.resources import *

from lib.soc.cores.rs232 import *
from lib.soc.interconnect import stream

from porting.litex.soc.cores.clock import S7PLL
from liteiclink.transceiver.gtp_7series import GTPQuadPLL, GTP

class Platform(Xilinx7SeriesPlatform):
    device      = "xc7a35t"
    package     = "fgg484"
    speed       = "2L"
    default_clk = "clk100"
    default_rst = "rst"
    resources   = [
        Resource("rst", 0, PinsN("AA1", dir="i"), Attrs(IOSTANDARD="LVCMOS33")),
        Resource("clk100", 0, Pins("R4", dir="i"),
                 Clock(100e6), Attrs(IOSTANDARD="LVCMOS33")),

        *LEDResources(pins="AB1 AB8", attrs=Attrs(IOSTANDARD="LVCMOS33")),
        *ButtonResources(pins="AA1 AB6", attrs=Attrs(IOSTANDARD="LVCMOS33")),

        UARTResource(0,
            rx="U1", tx="T1",
            attrs=Attrs(IOSTANDARD="LVCMOS33")
        ),

        Resource("pcie_tx", 0, DiffPairs("B6", "A6", dir="o")),
        Resource("pcie_rx", 0, DiffPairs("B10", "A10", dir="i")),
    ]
    connectors  = []

    def toolchain_prepare(self, fragment, name, **kwargs):
        overrides = {
            "add_constraints":
                "set_property SEVERITY {Warning} [get_drc_checks REQP-49]\n"
                # XXX PO hardcoded below
                "create_clock -name tx_clk -period 16.0 [get_nets tx_clk]\n"
                "create_clock -name rx_clk -period 16.0 [get_nets rx_clk]\n"
                "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets crg_clk100_0__i]] -group [get_clocks -include_generated_clocks -of [get_nets tx_clk]] -asynchronous\n"
                "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets crg_clk100_0__i]] -group [get_clocks -include_generated_clocks -of [get_nets rx_clk]] -asynchronous\n"
                "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets tx_clk]] -group [get_clocks -include_generated_clocks -of [get_nets rx_clk]] -asynchronous\n"
        }
        return super().toolchain_prepare(fragment, name, **overrides, **kwargs)


class _CRG(Elaboratable):
    def __init__(self, clk, rst):
        self.clk = clk
        self.rst = rst

    def elaborate(self, platform):
        m = Module()

        m.domains.sync = ClockDomain()
        m.domains.clk125 = cd_clk125 = ClockDomain("clk125")

        # # #

        m.d.comb += ClockSignal().eq(self.clk)
        m.submodules.reset_sync = ResetSynchronizer(self.rst, domain="sync")

        m.submodules.pll = pll = S7PLL()
        pll.register_clkin(self.clk, 100e6)
        pll.create_clkout(cd_clk125, 125e6)

        return m


class GTPTestTop(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        sys_clk_freq = int(100e6)
        clk100 = platform.request("clk100")
        rst = platform.request("rst")
        m.submodules += _CRG(clk100.i, rst.i)

        # refclk
        refclk = Signal()
        m.d.comb += refclk.eq(ClockSignal("clk125"))

        # pll
        qpll = GTPQuadPLL(refclk, 125e6, 1.25e9)
        print(qpll)
        m.submodules += qpll

        # gtp
        tx_pads = platform.request("pcie_tx", 0, dir="-")
        rx_pads = platform.request("pcie_rx", 0, dir="-")
        gtp = GTP(qpll, tx_pads, rx_pads, sys_clk_freq,
            data_width=20,
            clock_aligner=False,
            tx_buffer_enable=True,
            rx_buffer_enable=True)
        gtp.add_stream_endpoints()
        m.submodules += gtp

        # uart
        uart = RS232PHY(platform.request("uart"), sys_clk_freq, baudrate=115200)
        m.submodules += uart

        # cdctx
        cdctx = stream.AsyncFIFO([("data", 8)], 8, r_domain="tx", w_domain="sync")
        m.submodules += cdctx
        m.d.comb += uart.source.connect(cdctx.sink)

        # cdcrx
        cdcrx = stream.AsyncFIFO([("data", 8)], 8, r_domain="sync", w_domain="rx")
        m.submodules += cdcrx
        m.d.comb += cdcrx.source.connect(uart.sink)

        # counter
        counter = Signal(32)
        m.d.tx += counter.eq(counter + 1)

        # tx path
        with m.If(cdctx.source.valid):
            m.d.comb += [
                cdctx.source.ready.eq(1),
                gtp.sink.valid.eq(1), # gtp.sink always ready
                gtp.sink.ctrl.eq(0b00),
                gtp.sink.data[0:8].eq(0),
                gtp.sink.data[8:16].eq(cdctx.source.data),
            ]
        with m.Else():
            # K28.5 and slow counter --> TX
            m.d.comb += [
                gtp.sink.valid.eq(1),
                gtp.sink.ctrl.eq(0b01),
                gtp.sink.data[0:8].eq((5 << 5) | 28),
                gtp.sink.data[8:16].eq(counter[26:]),
            ]

        # rx path
        m.d.rx += [
            gtp.rx_align.eq(1),
            gtp.source.ready.eq(1),
        ]
        with m.If(gtp.source.ctrl == 0): # gtp.source always valid
            m.d.rx += [
                cdcrx.sink.valid.eq(1), # assume fifo ready
                cdcrx.sink.data.eq(gtp.source.data[8:16]),
            ]
        with m.Else():
            # RX (slow counter) --> Leds
            m.d.rx += [
                cdcrx.sink.valid.eq(0),
                platform.request("led", 0).eq(gtp.source.data[8]),
                platform.request("led", 1).eq(gtp.source.data[9]),
            ]

        # self.crg.cd_sync.clk.attr.add("keep")
        # gtp.cd_tx.clk.attr.add("keep")
        # gtp.cd_rx.clk.attr.add("keep")
        # platform.add_period_constraint(gtp.cd_tx.clk, 1e9/gtp.tx_clk_freq)
        # platform.add_period_constraint(gtp.cd_rx.clk, 1e9/gtp.rx_clk_freq)
        # platform.add_false_path_constraints(
        #     self.crg.cd_sync.clk,
        #     gtp.cd_tx.clk,
        #     gtp.cd_rx.clk)

        return m


def main():
    platform = Platform()
    platform.build(GTPTestTop(), name="pcie_screamer")


if __name__ == "__main__":
    main()
