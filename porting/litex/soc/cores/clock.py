# This file is Copyright (c) 2018-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# This file is Copyright (c) 2019 Michael Betz <michibetz@gmail.com>
# License: BSD

"""Clock Abstraction Modules"""

from nmigen.compat import *
from nmigen.compat.genlib.resetsync import AsyncResetSynchronizer


def period_ns(freq):
    return 1e9/freq

# Xilinx / Generic ---------------------------------------------------------------------------------

class XilinxClocking(Module):
    clkfbout_mult_frange = (2,  64+1)
    clkout_divide_range  = (1, 128+1)

    def __init__(self, vco_margin=0):
        self.vco_margin = vco_margin
        self.reset      = Signal()
        self.locked     = Signal()
        self.clkin_freq = None
        self.vcxo_freq  = None
        self.nclkouts   = 0
        self.clkouts    = {}
        self.config     = {}
        self.params     = {}

    def register_clkin(self, clkin, freq):
        self.clkin = Signal()
        from nmigen.hdl.ast import Signal as NativeSignal
        if isinstance(clkin, (Signal, ClockSignal, NativeSignal)):
            self.comb += self.clkin.eq(clkin)
        elif isinstance(clkin, Record):
            self.specials += DifferentialInput(clkin.p, clkin.n, self.clkin)
        else:
            raise ValueError
        self.clkin_freq = freq

    def create_clkout(self, cd, freq, phase=0, buf="bufg", margin=1e-2, with_reset=True):
        assert self.nclkouts < self.nclkouts_max
        clkout = Signal()
        self.clkouts[self.nclkouts] = (clkout, freq, phase, margin)
        self.nclkouts += 1
        if with_reset:
            self.specials += AsyncResetSynchronizer(cd, ~self.locked | self.reset)
        if buf is None:
            self.comb += cd.clk.eq(clkout)
        else:
            clkout_buf = Signal()
            self.comb += cd.clk.eq(clkout_buf)
            if buf == "bufg":
                self.specials += Instance("BUFG", i_I=clkout, o_O=clkout_buf)
            elif buf == "bufr":
                self.specials += Instance("BUFR", i_I=clkout, o_O=clkout_buf)
            else:
                raise ValueError

    def compute_config(self):
        config = {}
        for divclk_divide in range(*self.divclk_divide_range):
            config["divclk_divide"] = divclk_divide
            for clkfbout_mult in reversed(range(*self.clkfbout_mult_frange)):
                all_valid = True
                vco_freq = self.clkin_freq*clkfbout_mult/divclk_divide
                (vco_freq_min, vco_freq_max) = self.vco_freq_range
                if (vco_freq >= vco_freq_min*(1 + self.vco_margin) and
                    vco_freq <= vco_freq_max*(1 - self.vco_margin)):
                    for n, (clk, f, p, m) in sorted(self.clkouts.items()):
                        valid = False
                        for d in range(*self.clkout_divide_range):
                            clk_freq = vco_freq/d
                            if abs(clk_freq - f) < f*m:
                                config["clkout{}_freq".format(n)]   = clk_freq
                                config["clkout{}_divide".format(n)] = d
                                config["clkout{}_phase".format(n)]  = p
                                valid = True
                                break
                        if not valid:
                            all_valid = False
                else:
                    all_valid = False
                if all_valid:
                    config["vco"] = vco_freq
                    config["clkfbout_mult"] = clkfbout_mult
                    return config
        raise ValueError("No PLL config found")

    def do_finalize(self):
        assert hasattr(self, "clkin")

# Xilinx / 7-Series --------------------------------------------------------------------------------

class S7PLL(XilinxClocking):
    nclkouts_max = 6
    clkin_freq_range = (19e6, 800e6)

    def __init__(self, speedgrade=-1):
        XilinxClocking.__init__(self)
        self.divclk_divide_range = (1, 56+1)
        self.vco_freq_range = {
            -1: (800e6, 1600e6),
            -2: (800e6, 1866e6),
            -3: (800e6, 2133e6),
        }[speedgrade]

    def do_finalize(self):
        XilinxClocking.do_finalize(self)
        config = self.compute_config()
        pll_fb = Signal()
        self.params.update(
            p_STARTUP_WAIT="FALSE", o_LOCKED=self.locked,

            # VCO
            p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=1e9/self.clkin_freq,
            p_CLKFBOUT_MULT=config["clkfbout_mult"], p_DIVCLK_DIVIDE=config["divclk_divide"],
            i_CLKIN1=self.clkin, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,
        )
        for n, (clk, f, p, m) in sorted(self.clkouts.items()):
            self.params["p_CLKOUT{}_DIVIDE".format(n)] = config["clkout{}_divide".format(n)]
            self.params["p_CLKOUT{}_PHASE".format(n)] = config["clkout{}_phase".format(n)]
            self.params["o_CLKOUT{}".format(n)] = clk
        self.specials += Instance("PLLE2_ADV", **self.params)

# Lattice / ECP5 -----------------------------------------------------------------------------------

# TODO:
# - add proper phase support.

class ECP5PLL(Module):
    nclkouts_max    = 3
    clkfb_div_range = (1, 128+1)
    clko_div_range  = (1, 128+1)
    clki_freq_range = (    8e6,  400e6)
    clko_freq_range = (3.125e6,  400e6)
    vco_freq_range  = (  550e6, 1250e6)

    def __init__(self):
        self.reset      = Signal()
        self.locked     = Signal()
        self.clkin_freq = None
        self.vcxo_freq  = None
        self.nclkouts   = 0
        self.clkouts    = {}
        self.config     = {}
        self.params     = {}

    def register_clkin(self, clkin, freq):
        (clki_freq_min, clki_freq_max) = self.clki_freq_range
        assert freq >= clki_freq_min
        assert freq <= clki_freq_max
        self.clkin = Signal()
        from nmigen.hdl.ast import Signal as NativeSignal
        if isinstance(clkin, (Signal, ClockSignal, NativeSignal)):
            self.comb += self.clkin.eq(clkin)
        else:
            raise ValueError
        self.clkin_freq = freq

    def create_clkout(self, cd, freq, phase=0, margin=1e-2):
        (clko_freq_min, clko_freq_max) = self.clko_freq_range
        assert freq >= clko_freq_min
        assert freq <= clko_freq_max
        assert self.nclkouts < self.nclkouts_max
        clkout = Signal()
        self.clkouts[self.nclkouts] = (clkout, freq, phase, margin)
        self.nclkouts += 1
        self.comb += cd.clk.eq(clkout)

    def compute_config(self):
        config = {}
        config["clki_div"] = 1
        for clkfb_div in range(*self.clkfb_div_range):
            all_valid = True
            vco_freq = self.clkin_freq*clkfb_div*1 # clkos3_div=1
            (vco_freq_min, vco_freq_max) = self.vco_freq_range
            if vco_freq >= vco_freq_min and vco_freq <= vco_freq_max:
                for n, (clk, f, p, m) in sorted(self.clkouts.items()):
                    valid = False
                    for d in range(*self.clko_div_range):
                        clk_freq = vco_freq/d
                        if abs(clk_freq - f) < f*m:
                            config["clko{}_freq".format(n)] = clk_freq
                            config["clko{}_div".format(n)] = d
                            config["clko{}_phase".format(n)] = p
                            valid = True
                            break
                    if not valid:
                        all_valid = False
            else:
                all_valid = False
            if all_valid:
                config["vco"] = vco_freq
                config["clkfb_div"] = clkfb_div
                return config
        raise ValueError("No PLL config found")

    def do_finalize(self):
        config = self.compute_config()
        clkfb = Signal()
        self.params.update(
            a_ICP_CURRENT            = "6",
            a_LPF_RESISTOR           = "16",
            a_MFG_ENABLE_FILTEROPAMP = "1",
            a_MFG_GMCREF_SEL         = "2",

            i_RST=self.reset,

            i_CLKI = self.clkin,
            o_LOCK = self.locked,

            p_FEEDBK_PATH   = "INT_OS3",   # CLKOS3 reserved for
            p_CLKOS3_ENABLE = "ENABLED", # feedback with div=1.
            p_CLKOS3_DIV    = 1,

            p_CLKFB_DIV=config["clkfb_div"],
            p_CLKI_DIV=1,
        )
        for n, (clk, f, p, m) in sorted(self.clkouts.items()):
            n_to_l = {0: "P", 1: "S", 2: "S2"}
            self.params["p_CLKO{}_ENABLE".format(n_to_l[n])] = "ENABLED"
            self.params["p_CLKO{}_DIV".format(n_to_l[n])]    = config["clko{}_div".format(n)]
            self.params["p_CLKO{}_FPHASE".format(n_to_l[n])] = 0
            self.params["p_CLKO{}_CPHASE".format(n_to_l[n])] = p
            self.params["o_CLKO{}".format(n_to_l[n])]        = clk
        self.specials += Instance("EHXPLLL", **self.params)
