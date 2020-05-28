#!/usr/bin/env python3

from nmigen import *
from nmigen.lib.cdc import ResetSynchronizer

from nmigen.build import *
from nmigen.vendor.lattice_ecp5 import *
from nmigen_boards.resources import *

from lib.soc.cores.rs232 import *
from lib.soc.interconnect import stream

from porting.litex.soc.cores.clock import ECP5PLL
from liteiclink.transceiver.serdes_ecp5 import SerDesECP5PLL, SerDesECP5


class ECPIX5Platform(LatticeECP5Platform):
    device      = "LFE5UM5G-45F"
    package     = "BG554"
    speed       = "8"
    default_clk = "clk100"
    default_rst = "rst"

    resources   = [
        Resource("rst", 0, PinsN("AB1", dir="i"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("clk100", 0, Pins("K23", dir="i"), Clock(100e6), Attrs(IO_TYPE="LVCMOS33")),

        RGBLEDResource(0, r="U21", g="W21", b="T24", attrs=Attrs(IO_TYPE="LVCMOS33")),
        RGBLEDResource(1, r="T23", g="R21", b="T22", attrs=Attrs(IO_TYPE="LVCMOS33")),
        RGBLEDResource(2, r="P21", g="R23", b="P22", attrs=Attrs(IO_TYPE="LVCMOS33")),
        RGBLEDResource(3, r="K21", g="K24", b="M21", attrs=Attrs(IO_TYPE="LVCMOS33")),

        UARTResource(0,
            rx="R26", tx="R24",
            attrs=Attrs(IO_TYPE="LVCMOS33", PULLMODE="UP")
        ),

        *SPIFlashResources(0,
            cs="AA2", clk="AE3", miso="AE2", mosi="AD2", wp="AF2", hold="AE1",
            attrs=Attrs(IO_TYPE="LVCMOS33")
        ),

        Resource("eth_rgmii", 0,
            Subsignal("rst",     PinsN("C13", dir="o")),
            Subsignal("mdio",    Pins("A13", dir="io")),
            Subsignal("mdc",     Pins("C11", dir="o")),
            Subsignal("tx_clk",  Pins("A12", dir="o")),
            Subsignal("tx_ctrl", Pins("C9", dir="o")),
            Subsignal("tx_data", Pins("D8 C8 B8 A8", dir="o")),
            Subsignal("rx_clk",  Pins("E11", dir="i")),
            Subsignal("rx_ctrl", Pins("A11", dir="i")),
            Subsignal("rx_data", Pins("B11 A10 B10 A9", dir="i")),
            Attrs(IO_TYPE="LVCMOS33")
        ),
        Resource("eth_int", 0, PinsN("B13", dir="i"), Attrs(IO_TYPE="LVCMOS33")),

        *SDCardResources(0,
            clk="P24", cmd="M24", dat0="N26", dat1="N25", dat2="N23", dat3="N21", cd="L22",
            # TODO
            # clk_fb="P25", cmd_dir="M23", dat0_dir="N24", dat123_dir="P26",
            attrs=Attrs(IO_TYPE="LVCMOS33"),
        ),

        Resource("ddr3", 0,
            Subsignal("clk",    DiffPairs("H3", "J3", dir="o"), Attrs(IO_TYPE="SSTL135D_I")),
            Subsignal("clk_en", Pins("P1", dir="o")),
            Subsignal("we",     PinsN("R3", dir="o")),
            Subsignal("ras",    PinsN("T3", dir="o")),
            Subsignal("cas",    PinsN("P2", dir="o")),
            Subsignal("a",      Pins("T5 M3 L3 V6 K2 W6 K3 L1 H2 L2 N1 J1 M1 K1", dir="o")),
            Subsignal("ba",     Pins("U6 N3 N4", dir="o")),
            Subsignal("dqs",    DiffPairs("V4 V1", "U5 U2", dir="io"), Attrs(IO_TYPE="SSTL135D_I")),
            Subsignal("dq",     Pins("T4 W4 R4 W5 R6 P6 P5 P4 R1 W3 T2 V3 U3 W1 T1 W2", dir="io")),
            Subsignal("dm",     Pins("J4 H5", dir="o")),
            Subsignal("odt",    Pins("L2", dir="o")),
            Attrs(IO_TYPE="SSTL135_I")
        ),

        Resource("hdmi", 0,
            Subsignal("rst",   PinsN("N6", dir="o")),
            Subsignal("scl",   Pins("C17", dir="io")),
            Subsignal("sda",   Pins("E17", dir="io")),
            Subsignal("pclk",  Pins("C1", dir="o")),
            Subsignal("vsync", Pins("A4", dir="o")),
            Subsignal("hsync", Pins("B4", dir="o")),
            Subsignal("de",    Pins("A3", dir="o")),
            Subsignal("d",
                Subsignal("b", Pins("AD25 AC26 AB24 AB25  B3  C3  D3  B1  C2  D2 D1 E3", dir="o")),
                Subsignal("g", Pins("AA23 AA22 AA24 AA25  E1  F2  F1 D17 D16 E16 J6 H6", dir="o")),
                Subsignal("r", Pins("AD26 AE25 AF25 AE26 E10 D11 D10 C10  D9  E8 H5 J4", dir="o")),
            ),
            Subsignal("mclk",  Pins("E19", dir="o")),
            Subsignal("sck",   Pins("D6", dir="o")),
            Subsignal("ws",    Pins("C6", dir="o")),
            Subsignal("i2s",   Pins("A6 B6 A5 C5", dir="o")),
            Subsignal("int",   PinsN("C4", dir="i")),
            Attrs(IO_TYPE="LVTTL33")
        ),

        Resource("sata", 0,
            Subsignal("tx", DiffPairs("AD16", "AD17", dir="o")),
            Subsignal("rx", DiffPairs("AF15", "AF16", dir="i")),
            Attrs(IO_TYPE="LVDS")
        ),

        Resource("ulpi", 0,
            Subsignal("rst",  Pins("E23", dir="o")),
            Subsignal("clk",  Pins("H24", dir="i")),
            Subsignal("dir",  Pins("F22", dir="i")),
            Subsignal("nxt",  Pins("F23", dir="i")),
            Subsignal("stp",  Pins("H23", dir="o")),
            Subsignal("data", Pins("M26 L25 L26 K25 K26 J23 J26 H25", dir="io")),
            Attrs(IO_TYPE="LVCMOS33")
        ),

        Resource("usbc_cfg", 0,
            Subsignal("scl", Pins("D24", dir="io")),
            Subsignal("sda", Pins("C24", dir="io")),
            Subsignal("dir", Pins("B23", dir="i")),
            Subsignal("id",  Pins("D23", dir="i")),
            Subsignal("int", PinsN("B24", dir="i")),
            Attrs(IO_TYPE="LVCMOS33")
        ),
        Resource("usbc_mux", 0,
            Subsignal("en",    Pins("C23", dir="oe")),
            Subsignal("amsel", Pins("B26", dir="oe")),
            Subsignal("pol",   Pins("D26", dir="o")),
            # XXX
            # Subsignal("lna",   DiffPairs( "AF9", "AF10", dir="i"), Attrs(IO_TYPE="LVCMOS18D")),
            # Subsignal("lnb",   DiffPairs("AD10", "AD11", dir="o"), Attrs(IO_TYPE="LVCMOS18D")),
            # Subsignal("lnc",   DiffPairs( "AD7",  "AD8", dir="o"), Attrs(IO_TYPE="LVCMOS18D")),
            # Subsignal("lnd",   DiffPairs( "AF6",  "AF7", dir="i"), Attrs(IO_TYPE="LVCMOS18D")),
            Attrs(IO_TYPE="LVCMOS33")
        ),

        # XXX
        Resource("serdes_tx", 0, DiffPairs("AD10", "AD11", dir="o")),
        Resource("serdes_rx", 0, DiffPairs("AF9", "AF10", dir="i")),
    ]

    connectors  = [
        Connector("pmod", 0, "T25 U25 U24 V24 - - T26 U26 V26 W26 - -"),
        Connector("pmod", 1, "U23 V23 U22 V21 - - W25 W24 W23 W22 - -"),
        Connector("pmod", 2, "J24 H22 E21 D18 - - K22 J21 H21 D22 - -"),
        Connector("pmod", 3, " E4  F4  E6  H4 - -  F3  D4  D5  F5 - -"),
        Connector("pmod", 4, "E26 D25 F26 F25 - - A25 A24 C26 C25 - -"),
        Connector("pmod", 5, "D19 C21 B21 C22 - - D21 A21 A22 A23 - -"),
        Connector("pmod", 6, "C16 B17 C18 B19 - - A17 A18 A19 C19 - -"),
        Connector("pmod", 7, "D14 B14 E14 B16 - - C14 A14 A15 A16 - -"),
    ]

    @property
    def file_templates(self):
        return {
            **super().file_templates,
            "{{name}}-openocd.cfg": r"""
            interface ftdi
            ftdi_vid_pid 0x0403 0x6010
            ftdi_channel 0
            ftdi_layout_init 0xfff8 0xfffb
            reset_config none
            adapter_khz 25000

            jtag newtap ecp5 tap -irlen 8 -expected-id 0x81113043
            """
        }

    def toolchain_program(self, products, name):
        openocd = os.environ.get("OPENOCD", "openocd")
        with products.extract("{}-openocd.cfg".format(name), "{}.svf".format(name)) \
                as (config_filename, vector_filename):
            subprocess.check_call([openocd,
                "-f", config_filename,
                "-c", "transport select jtag; init; svf -quiet {}; exit".format(vector_filename)
            ])

    def toolchain_prepare(self, fragment, name, **kwargs):
        overrides = {
            "script_before_bitstream":
                "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]\n"
                "set_property BITSTREAM.CONFIG.CONFIGRATE 40 [current_design]",
            "script_after_bitstream":
                "write_cfgmem -force -format bin -interface spix4 -size 16 "
                "-loadbit \"up 0x0 {name}.bit\" -file {name}.bin".format(name=name),
            # "add_constraints":
                # "set_property INTERNAL_VREF 0.675 [get_iobanks 34]\n"
                # "set_property SEVERITY {Warning} [get_drc_checks REQP-49]\n"

                # XXX PO hardcoded below

                # XXX
                # "create_clock -name tx_clk -period 4.0 [get_nets tx_clk]\n"
                # "create_clock -name rx_clk -period 4.0 [get_nets rx_clk]\n"
                # "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets U$$0_clk100_0__i]] -group [get_clocks -include_generated_clocks -of [get_nets tx_clk]] -asynchronous\n"
                # "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets U$$0_clk100_0__i]] -group [get_clocks -include_generated_clocks -of [get_nets rx_clk]] -asynchronous\n"
                # "set_clock_groups -group [get_clocks -include_generated_clocks -of [get_nets tx_clk]] -group [get_clocks -include_generated_clocks -of [get_nets rx_clk]] -asynchronous\n"
        }
        return super().toolchain_prepare(fragment, name, **overrides, **kwargs)


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
    def __init__(self, linerate=5e9):
        assert linerate in [2.5e9, 5e9]
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

        # USB-C CC ---------------------------------------------------------------------------------
        usbc_cc = platform.request("usbc_cfg", 0)

        # USB-C MUX --------------------------------------------------------------------------------
        usbc_mux = platform.request("usbc_mux", 0)

        m.d.comb += [
            usbc_mux.pol.eq(usbc_cc.dir),
            usbc_mux.en.eq(1),
            usbc_mux.amsel.eq(0),
        ]

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
        tx_pads = platform.request("serdes_tx", 0, dir="-")
        rx_pads = platform.request("serdes_rx", 0, dir="-")
        channel = 1
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

        # Leds -------------------------------------------------------------------------------------
        rgbs = [platform.request("rgb_led", i) for i in range(4)]
        for rgb in rgbs[0:2]: # Green for iclink counter
            m.d.comb += [
               rgb.r.eq(1),
               rgb.b.eq(1),
            ]
        for rgb in rgbs[2:3]: # Off
            m.d.comb += [
               rgb.r.eq(1),
               rgb.g.eq(1),
               rgb.b.eq(1),
            ]
        for rgb in rgbs[3:]: # Blue for USB-CC dir
            m.d.comb += [
               rgb.r.eq(1),
               rgb.g.eq(1),
            ]
        m.d.comb += [
            rgbs[3].b.eq(usbc_cc.dir),
        ]

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
                rgbs[0].g.eq(~counter[0]),
                rgbs[1].g.eq(~counter[1]),
                # platform.request("rgb_led", 0).r.eq(~counter[2]),
                # platform.request("rgb_led", 1).r.eq(~counter[3]),
            ]

        # sys_counter = Signal(32)
        # m.d.sync += sys_counter.eq(sys_counter + 1)
        # m.d.comb += platform.request("led", 0).eq(sys_counter[26])

        # rx_counter = Signal(32)
        # m.d.rx += rx_counter.eq(rx_counter + 1)
        # m.d.comb += platform.request("led", 1).eq(rx_counter[26])

        # tx_counter = Signal(32)
        # m.d.tx += tx_counter.eq(rx_counter + 1)
        # m.d.comb += platform.request("led", 2).eq(tx_counter[26])

        return m


def main():
    platform = ECPIX5Platform()
    platform.build(SerDesTestTop(linerate=5e9), name="ecpix5")

if __name__ == "__main__":
    main()
