#!/usr/bin/env python3

from nmigen.compat import *
from nmigen.compat.genlib.resetsync import AsyncResetSynchronizer

from nmigen.build import *
from nmigen.vendor.xilinx_7series import *
from nmigen_boards.resources import *

from porting.litex.soc.cores.clock import *

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
            rx="AA6", tx="Y6",
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
                "create_clock -name tx_clk -period 16.0 [get_nets rx_clk]\n"
                "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets crg_clk100_0__i]] -group [get_clocks -include_generated_clocks -of [get_nets tx_clk]] -asynchronous\n"
                "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets crg_clk100_0__i]] -group [get_clocks -include_generated_clocks -of [get_nets rx_clk]] -asynchronous\n"
                "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets tx_clk]] -group [get_clocks -include_generated_clocks -of [get_nets rx_clk]] -asynchronous\n"
        }
        return super().toolchain_prepare(fragment, name, **overrides, **kwargs)


class _CRG(Module):
    def __init__(self, clk, rst):
        self.clock_domains.cd_sync = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        self.comb += self.cd_sync.clk.eq(clk)
        self.specials += AsyncResetSynchronizer(self.cd_sync, rst)

        self.submodules.pll = pll = S7PLL()
        pll.register_clkin(clk.i, 100e6)
        pll.create_clkout(self.cd_clk125, 125e6)


class GTPTestTop(Module):
    def __init__(self, platform):
        sys_clk_freq = int(100e6)
        clk100 = platform.request("clk100")
        rst = platform.request("rst", 0)
        self.submodules.crg = _CRG(clk100, rst)

        # refclk
        refclk = Signal()
        self.comb += refclk.eq(ClockSignal("clk125"))

        # pll
        qpll = GTPQuadPLL(refclk, 125e6, 1.25e9)
        print(qpll)
        self.submodules += qpll

        # gtp
        tx_pads = platform.request("pcie_tx", 0, dir="-")
        rx_pads = platform.request("pcie_rx", 0, dir="-")
        gtp = GTP(qpll, tx_pads, rx_pads, sys_clk_freq,
            data_width=20,
            clock_aligner=False,
            tx_buffer_enable=True,
            rx_buffer_enable=True)
        self.submodules += gtp

        # led blink
        counter = Signal(32)
        self.sync.tx += counter.eq(counter + 1)

        self.comb += [
            gtp.encoder.k[0].eq(1),
            gtp.encoder.d[0].eq((5 << 5) | 28),
            gtp.encoder.k[1].eq(0),
            gtp.encoder.d[1].eq(counter[26:]),
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

        self.comb += platform.request("led", 0).eq(gtp.decoders[1].d[0])
        self.comb += platform.request("led", 1).eq(gtp.decoders[1].d[1])

def main():
    platform = Platform()
    top = GTPTestTop(platform)
    platform.build(top, name="pcie_screamer")


if __name__ == "__main__":
    main()
