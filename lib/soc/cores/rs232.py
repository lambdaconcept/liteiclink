from nmigen import *
from nmigen.lib.cdc import *

from ..interconnect import stream


__all__ = ["RS232PHY"]


class RS232PHYRX(Elaboratable):
    def __init__(self, pads, tuning_word):
        self.source = stream.Endpoint([("data", 8)])
        self.pads = pads
        self.tuning_word = tuning_word

    def elaborate(self, platform):
        m = Module()

        uart_clk_rxen = Signal()
        phase_accumulator_rx = Signal(32)

        rx = Signal()
        m.submodules += FFSynchronizer(self.pads.rx, rx)
        rx_r = Signal()
        rx_reg = Signal(8)
        rx_bitcount = Signal(4)
        rx_busy = Signal()
        rx_done = self.source.valid
        rx_data = self.source.data

        m.d.sync += [
            rx_done.eq(0),
            rx_r.eq(rx)
        ]
        with m.If(~rx_busy):
            with m.If(~rx & rx_r): # look for start bit
                m.d.sync += [
                    rx_busy.eq(1),
                    rx_bitcount.eq(0)
                ]
        with m.Else():
            with m.If(uart_clk_rxen):
                m.d.sync += rx_bitcount.eq(rx_bitcount + 1)
                with m.If(rx_bitcount == 0):
                    with m.If(rx): # verify start bit
                        m.d.sync += rx_busy.eq(0)
                with m.Elif(rx_bitcount == 9):
                    m.d.sync += rx_busy.eq(0)
                    with m.If(rx): # verify stop bit
                        m.d.sync += [
                            rx_data.eq(rx_reg),
                            rx_done.eq(1)
                        ]
                with m.Else():
                    m.d.sync += rx_reg.eq(Cat(rx_reg[1:], rx))

        with m.If(rx_busy):
            m.d.sync += Cat(phase_accumulator_rx, uart_clk_rxen).eq(phase_accumulator_rx + self.tuning_word)
        with m.Else():
            m.d.sync += Cat(phase_accumulator_rx, uart_clk_rxen).eq(2**31)

        return m


class RS232PHYTX(Elaboratable):
    def __init__(self, pads, tuning_word):
        self.sink = stream.Endpoint([("data", 8)])
        self.pads = pads
        self.tuning_word = tuning_word

    def elaborate(self, platform):
        m = Module()

        uart_clk_txen = Signal()
        phase_accumulator_tx = Signal(32)

        self.pads.tx.reset = 1

        tx_reg = Signal(8)
        tx_bitcount = Signal(4)
        tx_busy = Signal()

        m.d.sync += self.sink.ready.eq(0)
        with m.If(self.sink.valid & ~tx_busy & ~self.sink.ready):
            m.d.sync += [
                tx_reg.eq(self.sink.data),
                tx_bitcount.eq(0),
                tx_busy.eq(1),
                self.pads.tx.eq(0)
            ]
        with m.Elif(uart_clk_txen & tx_busy):
            m.d.sync += tx_bitcount.eq(tx_bitcount + 1)
            with m.If(tx_bitcount == 8):
                m.d.sync += self.pads.tx.eq(1)
            with m.Elif(tx_bitcount == 9):
                m.d.sync += [
                    self.pads.tx.eq(1),
                    tx_busy.eq(0),
                    self.sink.ready.eq(1)
                ]
            with m.Else():
                m.d.sync += [
                    self.pads.tx.eq(tx_reg[0]),
                    tx_reg.eq(Cat(tx_reg[1:], 0))
                ]

        with m.If(tx_busy):
            m.d.sync += Cat(phase_accumulator_tx, uart_clk_txen).eq(phase_accumulator_tx + self.tuning_word)
        with m.Else():
            m.d.sync += Cat(phase_accumulator_tx, uart_clk_txen).eq(0)

        return m


class RS232PHY(Elaboratable):
    def __init__(self, pads, clk_freq, baudrate=115200):
        tuning_word = Signal(32, reset=int((baudrate/clk_freq)*2**32))
        self.tx = RS232PHYTX(pads, tuning_word)
        self.rx = RS232PHYRX(pads, tuning_word)
        self.sink, self.source = self.tx.sink, self.rx.source

    def elaborate(self, platform):
        m = Module()
        m.submodules += self.tx, self.rx
        return m
