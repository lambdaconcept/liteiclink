#!/usr/bin/env python3

# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteiclink.transceiver.gtp_7series import GTPQuadPLL, GTP


_io = [
    ("clk100", 0, Pins("R4"), IOStandard("LVCMOS33")),
    ("rst_n", Pins("AA1"), IOStandard("LVCMOS33")),

    ("user_led", 0, Pins("AB1"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("AB8"), IOStandard("LVCMOS33")),

    ("user_btn", 0, Pins("AA1"), IOStandard("LVCMOS33")),
    ("user_btn", 1, Pins("AB6"), IOStandard("LVCMOS33")),

    ("serial", 0,
        Subsignal("tx", Pins("Y6")),
        Subsignal("rx", Pins("AA6")),
        IOStandard("LVCMOS33")
    ),

    ("pcie_tx", 0,
        Subsignal("p", Pins("B6")),
        Subsignal("n", Pins("A6")),
    ),
    ("pcie_rx", 0,
        Subsignal("p", Pins("B10")),
        Subsignal("n", Pins("A10")),
    ),
]


class Platform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a35t-fgg484-2", _io, toolchain="vivado")

class _CRG(Module):
    def __init__(self, clk, rst):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        self.comb += self.cd_sys.clk.eq(clk)
        self.specials += AsyncResetSynchronizer(self.cd_sys, rst)

        self.submodules.pll = pll = S7PLL()
        pll.register_clkin(clk, 100e6)
        pll.create_clkout(self.cd_clk125, 125e6)


class GTPTestSoC(SoCCore):
    def __init__(self, platform):
        sys_clk_freq = int(100e6)
        SoCCore.__init__(self, platform, sys_clk_freq, cpu_type=None)
        clk100 = platform.request("clk100")
        rst = ~platform.request("user_btn", 0)
        self.submodules.crg = _CRG(clk100, rst)

        # refclk
        refclk = Signal()
        self.comb += refclk.eq(ClockSignal("clk125"))
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # pll
        qpll = GTPQuadPLL(refclk, 125e6, 1.25e9)
        print(qpll)
        self.submodules += qpll

        # gtp
        tx_pads = platform.request("pcie_tx", 0)
        rx_pads = platform.request("pcie_rx", 0)
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

        self.crg.cd_sys.clk.attr.add("keep")
        gtp.cd_tx.clk.attr.add("keep")
        gtp.cd_rx.clk.attr.add("keep")
        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/100e6)
        platform.add_period_constraint(gtp.cd_tx.clk, 1e9/gtp.tx_clk_freq)
        platform.add_period_constraint(gtp.cd_rx.clk, 1e9/gtp.rx_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtp.cd_tx.clk,
            gtp.cd_rx.clk)

        self.comb += platform.request("user_led", 0).eq(gtp.decoders[1].d[0])
        self.comb += platform.request("user_led", 1).eq(gtp.decoders[1].d[1])


def main():
    if "load" in sys.argv[1:]:
        from litex.build.xilinx import VivadoProgrammer
        prog = VivadoProgrammer()
        prog.load_bitstream("build/gateware/pcie_screamer.bit")
    else:
        platform = Platform()
        soc = GTPTestSoC(platform)
        builder = Builder(soc, output_dir="build", compile_gateware=True)
        vns = builder.build(build_name="pcie_screamer")


if __name__ == "__main__":
    main()
