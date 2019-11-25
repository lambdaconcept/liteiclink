# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from nmigen.compat import *

from litex.soc.interconnect import stream

from liteiclink.serwb.packet import Packetizer, Depacketizer
from liteiclink.serwb.etherbone import Etherbone


class SERWBCore(Module):
    def __init__(self, phy, clk_freq, mode):
        # etherbone
        self.submodules.etherbone = etherbone = Etherbone(mode)

        # packetizer / depacketizer
        depacketizer = Depacketizer(clk_freq)
        packetizer = Packetizer()
        self.submodules += depacketizer, packetizer

        # fifos
        tx_fifo = stream.SyncFIFO([("data", 32)], 8, buffered=True)
        rx_fifo = stream.SyncFIFO([("data", 32)], 8, buffered=True)
        self.submodules += tx_fifo, rx_fifo

        # modules connection
        self.comb += [
            # core --> phy
            packetizer.source.connect(tx_fifo.sink),
            tx_fifo.source.connect(phy.sink),

            # phy --> core
            phy.source.connect(rx_fifo.sink),
            rx_fifo.source.connect(depacketizer.sink),

            # etherbone <--> core
            depacketizer.source.connect(etherbone.sink),
            etherbone.source.connect(packetizer.sink)
        ]
