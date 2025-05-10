import asyncio
import bitstring
import math
import pigpio

from typing import Optional, List, Union, Final, Tuple

from .types import *

bitstring.options.lsb0 = True

_CS_PINS = {
    0: {0: 8, 1: 7},
    1: {0: 18, 1: 17, 2: 16},
}


def bytearray_to_bitstring(data: bytearray) -> bitstring.BitArray:
    data = data.removeprefix(b' ')
    return bitstring.BitArray(uint=int.from_bytes(data, byteorder="big"), length=8)


class SPI:
    _WRITE_BURST = 0x40
    _READ_SINGLE = 0x80
    _READ_BURST = 0xC0

    _ALL_TYPES = Union[Config, Strobe, StatusRegister, PTR]

    def __init__(
            self,
            spi_channel: int,
            spi_cs_channel: int,
            host: str = None,
            port: int = None
    ):
        self._spi_channel: int = spi_channel
        self._spi_cs_channel: int = spi_cs_channel
        self._handle: Optional[int] = None

        # use network connection if provided
        self._pi = pigpio.pi(host, port, show_errors=True)

    @property
    def MISO(self):
        return 9 if self._spi_channel == 0 else 19

    @property
    def MOSI(self):
        return 10 if self._spi_channel == 0 else 20

    @property
    def SCLK(self):
        return 11 if self._spi_channel == 0 else 21

    @property
    def CS(self):
        return _CS_PINS[self._spi_channel][self._spi_cs_channel]

    @property
    def channel(self):
        return self._spi_channel

    @property
    def cs_channel(self):
        return self._spi_cs_channel

    def begin(self, baud: int) -> None:
        self._handle = self._pi.spi_open(self._spi_channel, baud, self._spi_cs_channel)

    def close(self) -> None:
        self._pi.spi_close(self._handle)
        self._pi.stop()

    def write_reg(self, addr: _ALL_TYPES, value: Union[int, bitstring.BitArray]) -> None:
        if isinstance(value, bitstring.BitArray):
            value = value.uint
        self._pi.spi_write(self._handle, [addr.value, value])

    def write_burst(self, addr: _ALL_TYPES, value: List[Union[int, bitstring.BitArray]]) -> None:
        for item, idx in enumerate(value):
            if isinstance(item, bitstring.BitArray):
                value[idx] = item.uint
        self._pi.spi_write(self._handle, [addr.value | self._WRITE_BURST] + value)

    def strobe(self, strobe: Strobe) -> None:
        self._pi.spi_write(self._handle, [strobe.value])

    def read_reg(self, addr: _ALL_TYPES) -> bitstring.BitArray:
        count, data = self._pi.spi_xfer(self._handle, [addr.value | self._READ_SINGLE, 0])
        return bytearray_to_bitstring(data[1:])

    def read_burst(self, addr: _ALL_TYPES, num: int) -> bytearray:
        count, data = self._pi.spi_xfer(self._handle, [addr.value | self._READ_BURST] + [0] * num)
        return data

    def read_status_reg(self, addr: StatusRegister) -> bitstring.BitArray:
        count, data = self._pi.spi_xfer(self._handle, [addr.value | self._READ_BURST, 0])
        # if int.from_bytes(data, byteorder="big") != 4096:
        #     print("Error reading status register")
        return bytearray_to_bitstring(data[1:])


class ReceivedPacket:
    def __init__(
            self,
            data: bytearray,
            rssi: int,
            crc: bool,
            lqi: int,
            packet_length: int
    ):
        self._length = packet_length
        self._data = data
        self._rssi = rssi
        self._crc = crc
        self._lqi = lqi

    @property
    def rssi(self):
        rssi = self._rssi
        if rssi >= 128:
            rssi = (rssi - 256) / 2 - 74
        else:
            rssi = rssi / 2 - 74

        return rssi

    @property
    def length(self):
        return self._length

    @property
    def lqi(self):
        return self._lqi

    @property
    def valid(self):
        return self._crc

    @property
    def data(self):
        return self._data


class CC1101:
    # ToDo: add a way to check the RX Fifo
    # ToDo: hold Tx/Rx state in memory
    _XOSC_FREQ = 26e6

    def __init__(
            self,
            spi_channel: int = 0,
            spi_cs_channel: int = 0,
            gdo0_pin: int = 25,
            gdo2_pin: int = 7,
            host: str = 'localhost',
            port: int = 8888,
    ) -> None:
        self._spi = SPI(spi_channel, spi_cs_channel, host, port)
        self._modulation: Modulation = Modulation.GFSK
        self._state = State.IDLE

        self._gdo0_pin = gdo0_pin
        self._gdo2_pin = gdo2_pin
        # self._spi._pi.set_mode(gdo0_pin, pigpio.INPUT)
        # self._spi._pi.set_mode(gdo2_pin, pigpio.INPUT)

    def _set_defaults(self):
        self._spi.write_reg(Config.MCSM0, 20)

    def begin(self, baud: int = 57600) -> bool:
        self._spi.begin(baud)
        return bool(self._spi.read_status_reg(StatusRegister.VERSION))

    def close(self):
        self._spi.close()

    def calibrate(self):
        self._spi.strobe(Strobe.SCAL)

    def idle(self):
        self._spi.strobe(Strobe.SIDLE)
        self._state = State.IDLE

    async def send_data(self, payload: Union[str, int, bytes]):
        self.idle()
        if isinstance(payload, str):
            payload = payload.encode()
        elif isinstance(payload, int):
            payload = payload.to_bytes(length=(-(-payload // 255)), byteorder="big")

        packet_length_mode = self.get_packet_length_mode()
        if packet_length_mode == PacketLengthMode.VARIABLE:
            payload = [len(payload)] + list(payload)
        else:
            payload = list(payload)

        # self._spi.write_reg(PTR.TXFIFO, len(payload))
        self._spi.write_burst(PTR.TXFIFO, payload)
        self._spi.strobe(Strobe.STX)
        self._state = State.TX

        while self._spi.read_status_reg(StatusRegister.MARCSTATE).uint != 0x01:
            # print(self._spi.read_status_reg(StatusRegister.MARCSTATE).uint)
            await asyncio.sleep(0.001)

        await asyncio.sleep(0.01)
        self._spi.strobe(Strobe.SFTX)
        self.idle()

    # async def receive_data(self):
    #     # ToDo: make this stop
    #     self.idle()
    #     self._spi.strobe(Strobe.SRX)
    #     self._state = State.RX
    #     while True:
    #         if self._spi.read_reg(StatusRegister.RXBYTES) & 0x7F:
    #             length = self._spi.read_reg(PTR.RXFIFO).uint
    #             data = self._spi.read_burst(PTR.RXFIFO, length)
    #             self._spi.strobe(Strobe.SFRX)
    #
    #             self.idle()
    #             # ToDo: make sure putting the chip in idle state doesn't make it leak
    #             return ReceivedPacket(data)
    #         await asyncio.sleep(0.01)

    def set_mode(self, mode: State):
        if mode != self._state:
            if mode == State.RX:
                self.idle()
                self._spi.strobe(Strobe.SRX)
                self._state = State.RX
            elif mode == State.IDLE:
                self.idle()

    def receive_data(self) -> Optional[ReceivedPacket]:
        """Receive data from the RX FiFo"""
        # print(self.check_rx_fifo())
        if not self._state == State.RX:
            self._spi.strobe(Strobe.SRX)
            self._state = State.RX
        if self._state != State.RX:
            raise NotImplementedError
        if self.check_rx_fifo():
            # print(self._spi.read_status_reg(StatusRegister.MARCSTATE))
            length = self._spi.read_reg(PTR.RXFIFO).uint
            data = self._spi.read_burst(PTR.RXFIFO, length)
            self._spi.strobe(Strobe.SFRX)
            self._state = State.IDLE

            rssi = (self._spi.read_status_reg(StatusRegister.RSSI)).uint
            crc_lqi = self._spi.read_status_reg(StatusRegister.LQI)
            crc = crc_lqi[7]
            lqi = crc_lqi[:7].uint
            packet_length_mode = self.get_packet_length_mode()
            variable = True if packet_length_mode == PacketLengthMode.VARIABLE else False
            if variable is True:
                packet_length = data[0]
                data = data[1:]
            else:
                packet_length = self.get_packet_length()
            # print(packet_length)
            return ReceivedPacket(data, rssi, crc, lqi, packet_length)
        return None

    def check_rx_fifo(self):
        """Whether there is data in the RX FiFo"""
        if self._state != State.RX:
            # ToDo: alarm users of wrong state
            raise NotImplementedError

        # if self._spi._pi.read(self._gdo2_pin) == 1:
        #     return True
        # print(self._spi.read_status_reg(StatusRegister.RXBYTES).uint)
        # print(self._spi.read_status_reg(StatusRegister.MARCSTATE).uint)
        if self._spi.read_status_reg(StatusRegister.RXBYTES).uint & 0x7F:
            return True
        return False

    def reset(self):
        self._spi.strobe(Strobe.SRES)

    def set_cc_mode(self, cc_mode: bool) -> None:
        mdmcfg4 = self._spi.read_reg(Config.MDMCFG4)
        if cc_mode is True:
            self._spi.write_reg(Config.IOCFG2, 0x07)
            self._spi.write_reg(Config.IOCFG0, 0x06)
            self._spi.write_reg(Config.PKTCTRL0, 0x05)
            self._spi.write_reg(Config.MDMCFG3, 0xF8)
            self._spi.write_reg(Config.MDMCFG4, 11 + mdmcfg4[3:8].uint)
        else:
            self._spi.write_reg(Config.IOCFG2, 0x0D)
            self._spi.write_reg(Config.IOCFG0, 0x0D)
            self._spi.write_reg(Config.PKTCTRL0, 0x32)
            self._spi.write_reg(Config.MDMCFG3, 0x93)
            self._spi.write_reg(Config.MDMCFG4, 7 + mdmcfg4[3:8].uint)

    def set_modulation(self, modulation: Modulation) -> None:
        data = self._spi.read_reg(Config.MDMCFG2)
        data[4:7] = modulation.value
        self._spi.write_reg(Config.FREND0, 0x11 if modulation == Modulation.ASK else 0x10)
        self._spi.write_reg(Config.MDMCFG2, data)

    def get_modulation(self) -> Modulation:
        return Modulation((self._spi.read_reg(Config.MDMCFG2)[4:7]).uint)

    def set_dbm(self, dbm_level: int) -> None:
        pa_table = [0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0]
        if self._modulation == Modulation.FSK2:
            pa_table[0] = 0
            pa_table[1] = pa_table[dbm_level]
        else:
            pa_table[0] = pa_table[dbm_level]
            pa_table[1] = 0
        self._spi.write_burst(PTR.PATABLE, pa_table)

    def set_address(self, address: int):
        """
        Set the device address for sent and receive

        Args:
            address (int): The device address
        """
        if address > 256:
            raise ValueError
        self._spi.write_reg(Config.ADDR, address)

    def get_address(self) -> int:
        """
        Get the device address for sent and receive

        Returns:
            int: The device address
        """
        return self._spi.read_reg(Config.ADDR).uint

    def set_address_check(self, value: int):
        """
        Address check configuration of received packages

        Values:
            0 : No address check
            1 : Address check, no broadcast
            2 : Address check and 0 (0x00) broadcast
            3 : Address check and 0 (0x00) and 255 (0xFF) broadcast

        Args:
            value (int)
        """
        if value > 3:
            raise ValueError('Value is supposed to be 3 or lower')
        data = self._spi.read_reg(Config.PKTCTRL1)
        data[0:2] = value
        self._spi.write_reg(Config.PKTCTRL1, data)

    def get_address_check(self) -> int:
        """
        Current address check configuration
        """
        data = self._spi.read_reg(Config.PKTCTRL1)
        return data[0:2].uint

    def set_append_status(self, enable: bool):
        """
        When enabled, two status bytes will be appended to the payload of the packet.
        The status bytes contain RSSI and LQI values, as well as CRC OK.

        Args:
            enable (bool)
        """
        data = self._spi.read_reg(Config.PKTCTRL1)
        data[2] = enable
        self._spi.write_reg(Config.PKTCTRL1, data)

    def get_append_status(self) -> bool:
        """
        Whether append status is turned on

        Returns:
            bool
        """
        data = self._spi.read_reg(Config.PKTCTRL1)
        return data[2]

    def set_base_frequency(self, frequency: float):
        """
        Set the base carrier frequency in kHz
        The actual frequency is: base_frequency + channel_num * channel_spacing

        Args:
            frequency (float)
        """
        f = round(frequency / 0.3967285157216339)  # Page 57
        f_reg_val = list(f.to_bytes(length=3, byteorder="big"))
        self._spi.write_burst(Config.FREQ2, f_reg_val)

    def get_base_frequency(self) -> float:
        """
        Get the current base frequency

        Returns:
            float
        """
        freq_bytes = self._spi.read_burst(Config.FREQ2, num=3)
        freq = int.from_bytes(freq_bytes, byteorder="big", signed=False)
        return freq * 0.3967285157216339

    def set_channel(self, channel: int):
        """
        Set the channel on which the CC1101 should send/receive
        0 and 255 are broadcast channels

        Args:
            channel (int)
        """
        if channel > 255:
            raise ValueError
        self._spi.write_reg(Config.CHANNR, channel)

    def get_channel(self) -> int:
        """
        Get the current channel

        Returns:
            int
        """
        return self._spi.read_reg(Config.CHANNR).uint

    def set_channel_spacing(self, spacing: float):
        if spacing < 25.390625 or spacing > 405.456543:
            raise ValueError
        data = self._spi.read_reg(Config.MDMCFG1)
        mdmcfg0 = 0
        chsp = 0

        for _ in range(5):
            if spacing <= 50.682068:
                spacing -= 25.390625
                spacing /= 0.0991825
                mdmcfg0 = round(spacing)
                break
            else:
                chsp += 1
                spacing /= 2

        data[0:2] = chsp
        self._spi.write_reg(Config.MDMCFG1, data)
        self._spi.write_reg(Config.MDMCFG0, mdmcfg0)

    def get_channel_spacing(self) -> float:
        csm = self._spi.read_reg(Config.MDMCFG0).uint
        data = self._spi.read_reg(Config.MDMCFG1)
        return (self._XOSC_FREQ / 2**18) * (256 + csm) * 2**data[0:2].uint

    def set_crc(self, enable: bool):
        """
        Enable CRC calculation in TX and CRC check in RX

        Args:
            enable (bool)
        """
        data = self._spi.read_reg(Config.PKTCTRL0)
        data[2] = enable
        self._spi.write_reg(Config.PKTCTRL0, data)

    def get_crc(self) -> bool:
        """
        Whether CRC calculation is turned on

        Returns:
            bool
        """
        data = self._spi.read_reg(Config.PKTCTRL0)
        return data[2]

    def set_crc_af(self, enable: bool):
        """
        Enable automatic flush of RX FIFO when CRC is not OK

        Args:
            enable (bool): Enable if True
        """
        data = self._spi.read_reg(Config.PKTCTRL1)
        data[3] = enable
        self._spi.write_reg(Config.PKTCTRL1, data)

    def get_crc_af(self) -> bool:
        """
        Whether auto flush of the RX FIFO when CRC is not OK is turned on

        Returns:
            bool
        """
        data = self._spi.read_reg(Config.PKTCTRL1)
        return data[3]

    def set_data_rate(self, data_rate: float):
        """
        Set the data rate in Baud
        Ranges:
            2-FSK: 0.6kBaud - 500kBaud
            GFSK, OOK, ASK: 0.6kBaud - 250kBaud
            4-FSK: 0.6kBaud - 300kBaud
            MSK: 26kBaud - 500kBaud

        Args:
            data_rate (float): The data rate in Baud
        """
        drate_e = int(math.log2(data_rate * 2**20 / self._XOSC_FREQ))
        drate_m = round((data_rate * 2**28) / (self._XOSC_FREQ * 2**drate_e) - 256)
        if drate_m == 256:
            drate_m = 0
            drate_e += 1

        data = self._spi.read_reg(Config.MDMCFG4)
        data[0:4] = drate_e
        self._spi.write_reg(Config.MDMCFG4, data)
        self._spi.write_reg(Config.MDMCFG3, drate_m)

    def get_data_rate(self) -> float:
        """
        Get the current data rate

        Returns:
            float
        """
        dm = self._spi.read_reg(Config.MDMCFG3).uint
        data = self._spi.read_reg(Config.MDMCFG4)
        de = data[0:4].uint
        dr = (((256 + dm) * 2**de) / 2**28) * self._XOSC_FREQ
        return dr

    def set_dc_filter(self, enable: bool):
        data = self._spi.read_reg(Config.MDMCFG2)
        data[7] = enable
        self._spi.write_reg(Config.MDMCFG2, data)

    def get_dc_filter(self) -> bool:
        data = self._spi.read_reg(Config.MDMCFG2)
        return data[7]

    def set_deviation(self, deviation: float):
        if deviation < 1586.914 or deviation > 380859.375:
            raise ValueError('Deviation should be between 1586.914 Hz and 380859.375 Hz')
        deviation_e = int(math.log2(deviation * 2**14 / self._XOSC_FREQ))
        deviation_m = round(deviation * 2**(17-deviation_e) / self._XOSC_FREQ) - 8
        deviation = bitstring.BitArray(uint=0, length=8)
        deviation[0:3] = deviation_m
        deviation[4:7] = deviation_e
        self._spi.write_reg(Config.DEVIATN, deviation)

    def get_deviation(self) -> float:
        d = self._spi.read_reg(Config.DEVIATN)
        return (self._XOSC_FREQ / 2**(17 - d[4:7].uint)) * (8 + d[0:3].uint)

    def set_sync_word(self, sh: int, sl: int):
        """
        Set the sync words

        Args:
            sh (int): Sync high
            sl (int): Sync low
        """
        if sh > 256 or sl > 256:
            raise ValueError
        self._spi.write_reg(Config.SYNC1, sh)
        self._spi.write_reg(Config.SYNC0, sl)

    def get_sync_word(self) -> List[int]:
        """
        Get the current sync words

        Returns:
            List[int, int]: The current sync words (sh, sl)
        """
        sync = self._spi.read_burst(Config.SYNC1, 2)
        return [sync[1], sync[2]]

    def set_freq_autocal(self, autocal: int):
        """
        Set when the CC1101 should auto calibrate

        Values:
            0: Never calibrate automatically
            1: Calibrate when going from IDLE to RX/TX
            2: Calibrate when going from RX/TX to IDLE
            3: Calibrate every 4th time when going from RX/TX to IDLE

        Args:
            autocal (int): When to auto calibrate
        """
        data = self._spi.read_reg(Config.MCSM1)
        data[4:6] = autocal
        self._spi.write_reg(Config.MCSM1, data)

    def set_pqt(self, threshold: int):
        """
        Set the preamble quality estimator threshold

        Args:
            threshold (int): The threshold (3bit integer)
        """
        if threshold > 7:
            raise ValueError("Threshold should be 6 or lower")
        data = self._spi.read_reg(Config.PKTCTRL1)
        data[5:8] = threshold
        self._spi.write_reg(Config.PKTCTRL1, data)

    def get_pqt(self) -> int:
        """
        Get the current preamble quality estimator

        Returns:
            int: The current preamble quality estimator
        """
        data = self._spi.read_reg(Config.PKTCTRL1)
        return data[5:8].uint

    def set_rxend_behavior(self, value: int):
        """
        Set what to do on the end of a successful packet receive
        Check the datasheet https://www.ti.com/lit/ds/swrs061i/swrs061i.pdf on page 28 for details

        ToDo: cc1101.state is currently unreliable if anything else but 0 is used

        Values:
            0: IDLE
            1: FSTXON
            2: TX
            3: Stay in RX

        Args:
            value (int)
        """
        if value > 3:
            raise ValueError
        data = self._spi.read_reg(Config.MCSM1)
        data[2:4] = value
        self._spi.write_reg(Config.MCSM1, data)

    def get_rxend_behavior(self):
        """
        Get the current RX end behavior
        """
        data = self._spi.read_reg(Config.MCSM1)
        return data[2:4].uint

    def set_txend_behavior(self, value: int):
        """
        Set what to do on the end of a successful packet transmit
        Check the datasheet https://www.ti.com/lit/ds/swrs061i/swrs061i.pdf on page 28 for details

        Values:
            0: IDLE
            1: FSTXON
            2: Stay in TX
            3: RX

        Args:
            value (int)
        """
        if value > 3:
            raise ValueError
        data = self._spi.read_reg(Config.MCSM1)
        data[0:2] = value
        self._spi.write_reg(Config.MCSM1, data)

    def get_txend_behavior(self):
        """
        Get the current TX end behavior
        """
        data = self._spi.read_reg(Config.MCSM1)
        return data[0:2].uint

    def set_white_data(self, enable: bool):
        """
        Turn on data whitening using a pseudo-random sequence.
        Might be helpful for data with lots of repeating 0 or 1

        Args:
            enable (bool)
        """
        data = self._spi.read_reg(Config.PKTCTRL0)
        data[6] = enable
        self._spi.write_reg(Config.PKTCTRL0, data)

    def get_white_data(self) -> bool:
        """
        Whether data whitening is turned on

        Returns:
            bool
        """
        data = self._spi.read_reg(Config.PKTCTRL0)
        return data[6]

    def set_pkt_format(self, value: int):
        """
        Set the format of RX and TX data
        Currently only normal mode (0) is supported by this library

        Values:
            0 : Normal mode, use FIFOs for RX and TX
            1 : Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins
            2 : Random TX mode; sends random data using PN9 generator. Used for test.
                Works as normal mode, setting 0 (00), in RX
            3 : Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins

        Args:
            value (int)
        """
        if value > 3:
            raise ValueError
        data = self._spi.read_reg(Config.PKTCTRL0)
        data[4:6] = value
        self._spi.write_reg(Config.PKTCTRL0, data)

    def get_pkt_format(self) -> int:
        """
        Get the current packet format configuration

        Returns:
            int
        """
        data = self._spi.read_reg(Config.PKTCTRL0)
        return data[4:6].uint

    def set_packet_length_mode(self, mode: PacketLengthMode):
        data = self._spi.read_reg(Config.PKTCTRL0)
        data[0:2] = mode.value
        self._spi.write_reg(Config.PKTCTRL0, data)

    def get_packet_length_mode(self) -> int:
        data = self._spi.read_reg(Config.PKTCTRL0)
        return PacketLengthMode(data[0:2].uint)

    def set_packet_length(self, length: int):
        if length > 255:
            raise ValueError
        self._spi.write_reg(Config.PKTLEN, length)

    def get_packet_length(self) -> int:
        return self._spi.read_reg(Config.PKTLEN).uint

    def set_manchester(self, enable: bool):
        data = self._spi.read_reg(Config.MDMCFG2)
        data[3] = enable
        self._spi.write_reg(Config.MDMCFG2, data)

    def get_manchester(self) -> bool:
        data = self._spi.read_reg(Config.MDMCFG2)
        return data[3]

    def set_sync_mode(self, syncm: int):
        """
        Setting Sync-word qualifier mode
        0 (000) No preamble/sync
        1 (001) 15/16 sync word bits detected
        2 (010) 16/16 sync word bits detected
        3 (011) 30/32 sync word bits detected
        4 (100) No preamble/sync, carrier-sense above threshold
        5 (101) 15/16 + carrier-sense above threshold
        6 (110) 16/16 + carrier-sense above threshold
        7 (111) 30/32 + carrier-sense above threshold
        """
        if syncm > 7:
            syncm = 7
        data = self._spi.read_reg(Config.MDMCFG2)
        data[0:3] = syncm
        self._spi.write_reg(Config.MDMCFG2, data)

    def get_sync_mode(self) -> int:
        data = self._spi.read_reg(Config.MDMCFG2)
        return data[0:3].uint

    def set_fec(self, enable: bool):
        data = self._spi.read_reg(Config.MDMCFG1)
        data[7] = enable
        self._spi.write_reg(Config.MDMCFG1, data)

    def get_fec(self) -> bool:
        data = self._spi.read_reg(Config.MDMCFG1)
        return data[7]

    def set_preamble(self, value: int):
        """
        Set the amount of preamble bits

        Values:
            0: 2
            1: 3
            2: 4
            3: 6
            4: 8
            5: 12
            6: 16
            7: 24

        Args:
            value (int): The preamble value
        """
        if value > 7:
            value = 7
        data = self._spi.read_reg(Config.MDMCFG1)
        data[4:7] = value
        self._spi.write_reg(Config.MDMCFG1, data)

    def get_preamble(self) -> int:
        data = self._spi.read_reg(Config.MDMCFG1)
        return data[4:7].uint

    def set_rx_bandwidth(self, bandwidth: float):
        data = self._spi.read_reg(Config.MDMCFG4)

        rxbw_e = round(- math.log2(bandwidth * 8 * 5 / self._XOSC_FREQ))
        rxbw_m = round(self._XOSC_FREQ / (8 * bandwidth * 2**rxbw_e) - 4)

        data[4:6] = rxbw_m
        data[6:8] = rxbw_e
        self._spi.write_reg(Config.MDMCFG4, data)

    def get_rx_bandwidth(self) -> float:
        rxbw = self._spi.read_reg(Config.MDMCFG4)[4:8]
        return self._XOSC_FREQ / (8 * (4 + rxbw[0:2].uint) * 2**int(rxbw[2:4].uint))

    def get_rssi(self) -> int:
        rssi = self._spi.read_status_reg(StatusRegister.RSSI).uint
        if rssi >= 128:
            rssi = (rssi - 256) / 2 - 74
        else:
            rssi = rssi / 2 - 74

        return rssi

    def get_lqi(self) -> int:
        return self._spi.read_status_reg(StatusRegister.LQI).uint
