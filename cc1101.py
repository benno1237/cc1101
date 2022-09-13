import asyncio
import bitstring
import pigpio

from typing import Optional, List, Union, Final, Tuple

from .types import Config, Strobe, StatusRegister, PTR, Modulation, _CS_PINS, State

bitstring.set_lsb0(True)

class SPI:
    _WRITE_BURST = 0x40
    _READ_SINGLE = 0x80
    _READ_BURST = 0xC0

    _ALL_TYPES: Final[Union] = Union[Config, Strobe, StatusRegister, PTR]

    def __init__(
            self,
            spi_channel: int,
            spi_cs_channel: int,
    ):
        self._spi_channel: int = spi_channel
        self._spi_cs_channel: int = spi_cs_channel
        self._handle: Optional[int] = None

        self._pi = pigpio.pi()

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

    def write_reg(self, addr: _ALL_TYPES, value: Union[int, bitstring.BitArray]) -> None:
        if isinstance(value, bitstring.BitArray):
            value = value.uint
        self._pi.spi_write(self._handle, addr.value + value)

    def write_burst(self, addr: _ALL_TYPES, value: List[Union[int, bitstring.BitArray]]) -> None:
        for item, idx in enumerate(value):
            if isinstance(item, bitstring.BitArray):
                value[idx] = item.uint
        self._pi.spi_write(self._handle, [addr.value | self._WRITE_BURST] + value)

    def strobe(self, strobe: Strobe) -> None:
        self._pi.spi_write(self._handle, strobe.value)

    def read_reg(self, addr: _ALL_TYPES) -> bitstring.BitArray:
        count, data = self._pi.spi_xfer(self._handle, [addr.value | self._READ_SINGLE, 0])
        return bitstring.BitArray(int=data, length=8)

    def read_burst(self, addr: _ALL_TYPES, num: int) -> List[int]:
        count, data = self._pi.spi_xfer(self._handle, [addr.value | self._READ_BURST] + [0] * num)
        return data

    def read_status_reg(self, addr: StatusRegister) -> int:
        count, data = self._pi.spi_xfer(self._handle, [addr.value | self._READ_BURST, 0])
        return data


class ReceivedPacket:
    def __init__(
            self,
            data: list[int],
    ):
        self._data = data

    @property
    def rssi(self):
        rssi = self._data[1]
        if rssi >= 128:
            rssi = (rssi - 256) / 2 - 74
        else:
            rssi = rssi / 2 - 74

        return rssi

    @property
    def lqi(self):
        return (self._data[0] << 1 & 255) >> 1

    @property
    def valid(self):
        return self._data[0] >> 7

    @property
    def data(self):
        return self._data[2:]

class CC1101:
    # ToDo: add a way to check the RX Fifo
    # ToDo: hold Tx/Rx state in memory
    _XOSC_FREQ = 26e6

    def __init__(
            self,
            spi_channel: int = 0,
            spi_cs_channel: int = 0,
    ) -> None:
        self._spi = SPI(spi_channel, spi_cs_channel)
        self._modulation: Modulation = Modulation.GFSK
        self._state = State.IDLE

    def _split_pktctrl1(self) -> Tuple[int, int, int, int]:
        val = self._spi.read_reg(Config.PKTCTRL1)
        pqt, crc_af, app_st, adrchk = 0, 0, 0, 0
        while val >= 4:
            if val >= 32:
                pqt += 32
                val -= 32
            elif val >= 8:
                crc_af += 8
                val -= 8
            elif val >= 4:
                app_st += 4
                val -= 4
        else:
            adrchk = val
        return pqt, crc_af, app_st, adrchk

    def _split_pktctrl0(self) -> Tuple[int, int, int, int]:
        val = self._spi.read_reg(Config.PKTCTRL1)
        wdata, pktform, crc_en, lenconf = 0, 0, 0, 0
        while val >= 4:
            if val >= 64:
                wdata += 64
                val -= 64
            elif val >= 16:
                pktform += 16
                val -= 16
            elif val >= 4:
                crc_en += 4
                val -= 4
        else:
            lenconf = val
        return wdata, pktform, crc_en, lenconf

    def _split_mdmcfg1(self) -> Tuple[int, int, int]:
        val = self._spi.read_reg(Config.MDMCFG1)
        fec, pre, chsp = 0, 0, 0
        while val >= 16:
            if val >= 128:
                fec += 128
                val -= 128
            elif val >= 16:
                pre += 16
                val -= 16
        else:
            chsp = val
        return fec, pre, chsp

    def _split_mdmcfg2(self) -> Tuple[int, int, int, int]:
        val = self._spi.read_reg(Config.MDMCFG2)
        dcoff, modfm, manch, syncm = 0, 0, 0, 0
        while val >= 8:
            if val >= 128:
                dcoff += 128
                val -= 182
            elif val >= 16:
                modfm += 16
                val -= 16
            elif val >= 8:
                manch += 8
                val -= 8
        else:
            syncm = val
        return dcoff, modfm, manch, syncm

    def _split_mdmcfg4(self) -> Tuple[int, int]:
        val = self._spi.read_reg(Config.MDMCFG4)
        rxbw, dara = 0, 0
        while val >= 16:
            if val >= 64:
                rxbw += 64
                val -= 64
            elif val >= 16:
                rxbw += 16
                val -= 16
        else:
            dara = val
        return rxbw, dara

    def _set_defaults(self):
        self._spi.write_reg(Config.MCSM0, 20)

    def begin(self, kbaud: int) -> bool:
        self._spi.begin(kbaud * 1000)
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
        self._spi.write_burst(PTR.TXFIFO, list(payload))
        self._spi.strobe(Strobe.STX)
        self._state = State.TX
        while self._spi.read_reg(StatusRegister.MARCSTATE) != 0x01:
            await asyncio.sleep(0.001)
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

    def receive_data(self) -> Optional[ReceivedPacket]:
        """Receive data from the RX FiFo"""
        if self.check_rx_fifo():
            length = self._spi.read_reg(PTR.RXFIFO).uint
            data = self._spi.read_burst(PTR.RXFIFO, length)
            self._spi.strobe(Strobe.SFRX)
            return ReceivedPacket(data)
        return None

    def check_rx_fifo(self):
        """Whether there is data in the RX FiFo"""
        if self._state != State.RX:
            # ToDo: alarm users of wrong state
            raise NotImplementedError

        if self._spi.read_status_reg(StatusRegister.RXBYTES):
            return True
        return False

    async def reset(self):
        self._spi._pi.write(self._spi.CS, 0)
        await asyncio.sleep(0.02)
        self._spi._pi.write(self._spi.CS, 1)
        # while self.pi.read(self._spi.MISO):
        #     time.sleep(0.001)
        self._spi.strobe(Strobe.SRES)

    def set_cc_mode(self, cc_mode: bool) -> None:
        if cc_mode is True:
            self._spi.write_reg(Config.IOCFG2, 0x0B)
            self._spi.write_reg(Config.IOCFG0, 0x06)
            self._spi.write_reg(Config.PKTCTRL0, 0x05)
            self._spi.write_reg(Config.MDMCFG3, 0xF8)
            self._spi.write_reg(Config.MDMCFG4, 11 + self._split_mdmcfg4()[0])
        else:
            self._spi.write_reg(Config.IOCFG2, 0x0D)
            self._spi.write_reg(Config.IOCFG0, 0x0D)
            self._spi.write_reg(Config.PKTCTRL0, 0x32)
            self._spi.write_reg(Config.MDMCFG3, 0x93)
            self._spi.write_reg(Config.MDMCFG4, 7 + self._split_mdmcfg4()[0])

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

    def set_base_frequency(self, frequency: float):
        f = int(frequency / 0.0003967285157216339)
        f_reg_val = list(f.to_bytes(length=3, byteorder="big"))
        self._spi.write_burst(Config.FREQ2, f_reg_val)

    def get_base_frequency(self) -> float:
        freq_bytes = self._spi.read_burst(Config.FREQ2, num=3)
        freq = int.from_bytes(freq_bytes, byteorder="big", signed=False)
        return freq * 0.0003967285157216339

    def set_sync_word(self, sh: int, sl: int):
        if sh > 256 or sl > 256:
            print("Error")
        self._spi.write_reg(Config.SYNC1, sh)
        self._spi.write_reg(Config.SYNC0, sl)

    def get_sync_word(self) -> List[int, int]:
        return self._spi.read_burst(Config.SYNC1, 2)

    def set_address(self, address: int):
        if address > 256:
            print("Error")
        self._spi.write_reg(Config.ADDR, address)

    def get_address(self) -> int:
        return self._spi.read_reg(Config.ADDR).uint

    def set_pqt(self, threshold: int):
        if threshold > 7:
            print("Error")
        data = self._spi.read_reg(Config.PKTCTRL1)
        data[5:8] = threshold
        self._spi.write_reg(data)

    def get_pqt(self) -> int:
        pqt, *_ = self._split_pktctrl1()
        return pqt

    def set_crc_af(self, enable: bool):
        pqt, _, app_st, adrchk = self._split_pktctrl1()
        crc_af = 8 if enable is True else 0
        self._spi.write_reg(Config.PKTCTRL1, pqt+crc_af+app_st+adrchk)

    def get_crc_af(self) -> bool:
        _, crc_af, *_ = self._split_pktctrl1()
        return bool(crc_af)

    def set_append_status(self, enable: bool):
        pqt, crc_af, _, adrchk = self._split_pktctrl1()
        app_st = 4 if enable is True else 0
        self._spi.write_reg(Config.PKTCTRL1, pqt+crc_af+app_st+adrchk)

    def get_append_status(self) -> bool:
        _, _, app_st, _ = self._split_pktctrl1()
        return bool(app_st)

    def set_address_check(self, value: int):
        if value > 3:
            value = 3
        pqt, crc_af, app_st, _ = self._split_pktctrl1()
        adrchk = value
        self._spi.write_reg(Config.PKTCTRL1, pqt+crc_af+app_st+adrchk)

    def get_address_check(self) -> int:
        _, _, _, adrchk = self._split_pktctrl1()
        return adrchk

    def set_white_data(self, enable: bool):
        _, pktform, crc_en, lenconf = self._split_pktctrl0()
        wdata = 64 if enable is True else 0
        self._spi.write_reg(Config.PKTCTRL0, wdata+pktform+crc_en+lenconf)

    def get_white_data(self) -> bool:
        wdata, *_ = self._split_pktctrl0()
        return bool(wdata)

    def set_pkt_format(self, value: int):
        if value > 3:
            value = 3
        wdata, _, crc_en, lenconf = self._split_pktctrl0()
        pktform = value * 16
        self._spi.write_reg(Config.PKTCTRL0, wdata+pktform+crc_en+lenconf)

    def get_pkt_format(self) -> int:
        _, pktform, *_ = self._split_pktctrl0()
        return pktform

    def set_crc(self, enable: bool):
        wdata, pktform, _, lenconf = self._split_pktctrl0()
        crc_en = 4 if enable is True else 0
        self._spi.write_reg(Config.PKTCTRL0, wdata+pktform+crc_en+lenconf)

    def get_crc(self) -> bool:
        _, _, crc, _ = self._split_pktctrl0()
        return bool(crc)

    def set_length_config(self, value: int):
        if value > 3:
            value = 3
        wdata, pktform, crc_en, _ = self._split_pktctrl0()
        lenconf = value
        self._spi.write_reg(Config.PKTCTRL0, wdata+pktform+crc_en+lenconf)

    def get_length_config(self) -> int:
        _, _, _, lenconf = self._split_pktctrl0()
        return lenconf

    def set_packet_length(self, length: int):
        if length > 255:
            print("Error")
        self._spi.write_reg(Config.PKTLEN, length)

    def get_packet_length(self) -> int:
        return self._spi.read_reg(Config.PKTLEN)

    def set_dc_filter(self, enable: bool):
        _, modfm, manch, syncm = self._split_mdmcfg2()
        dcoff = 0 if enable is True else 128
        self._spi.write_reg(Config.MDMCFG2, dcoff+modfm+manch+syncm)

    def get_dc_filter(self) -> bool:
        dcoff, *_ = self._split_mdmcfg2()
        return not bool(dcoff)

    def set_manchester(self, enable: bool):
        dcoff, modfm, _, syncm = self._split_mdmcfg2()
        manch = 8 if enable is True else 0
        self._spi.write_reg(Config.MDMCFG2, dcoff+modfm+manch+syncm)

    def get_manchester(self) -> bool:
        _, _, manch, _ = self._split_mdmcfg2()
        return bool(manch)

    def set_sync_mode(self, syncm: int):
        if syncm > 7:
            syncm = 7
        dcoff, modfm, manch, _ = self._split_mdmcfg2()
        self._spi.write_reg(Config.MDMCFG2, dcoff+modfm+manch+syncm)

    def get_sync_mode(self) -> int:
        _, _, _, syncm = self._split_mdmcfg2()
        return syncm

    def set_fec(self, enable: bool):
        _, pre, chsp = self._split_mdmcfg1()
        fec = 128 if enable is True else 0
        self._spi.write_reg(Config.MDMCFG1, fec+pre+chsp)

    def get_fec(self) -> bool:
        fec, *_ = self._split_mdmcfg1()
        return bool(fec)

    def set_pre(self, value: int):
        if value > 7:
            value = 7
        fec, _, chsp = self._split_mdmcfg1()
        self._spi.write_reg(Config.MDMCFG1, fec+value*7+chsp)

    def get_pre(self) -> int:
        _, pre, _ = self._split_mdmcfg1()
        return pre

    def set_channel(self, channel: int):
        if channel > 255:
            print("Error")
        self._spi.write_reg(Config.CHANNR, channel)

    def set_channel_spacing(self, spacing: float):
        if spacing < 25.390625 or spacing > 405.456543:
            print("Error")
        _, fec, pre = self._split_mdmcfg1()
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

        self._spi.write_reg(Config.MDMCFG1, chsp+fec+pre)
        self._spi.write_reg(Config.MDMCFG0, mdmcfg0)

    def get_channel_spacing(self) -> float:
        csm = self._spi.read_reg(Config.MDMCFG0)
        cse, *_ = self._split_mdmcfg1()
        return (self._XOSC_FREQ / 2**18) * (256 + csm) * 2**cse

    def set_rx_bandwidth(self, bandwidth: float):
        _, dara = self._split_mdmcfg4()
        s1, s2 = 3, 3
        for _ in range(3):
            if bandwidth > 101.5625:
                bandwidth /= 2
                s1 -= 1
            else:
                break
        for _ in range(3):
            if bandwidth > 58.1:
                bandwidth /= 1.25
                s2 -= 1
            else:
                break
        rxbw = s1 * 64 + s2 * 16
        self._spi.write_reg(Config.MDMCFG4, rxbw+dara)

    def get_rx_bandwidth(self) -> float:
        rxbw = format(self._split_mdmcfg4()[0], "02X")
        return self._XOSC_FREQ / (8 * (4 + int(rxbw[1])) * 2**int(rxbw[0]))

    def set_data_rate(self, data_rate: float):
        if data_rate < 0.0247955 or data_rate > 1621.83:
            print("Error")
        rxbw, _ = self._split_mdmcfg4()
        dara = 0
        mdmcfg3 = 0
        for _ in range(20):
            if data_rate <= 0.049492:
                data_rate -= 0.0247955
                data_rate /= 0.00009685
                mdmcfg3 = round(data_rate)
                break
            else:
                dara += 1
                data_rate /= 2
        self._spi.write_reg(Config.MDMCFG4, rxbw+dara)
        self._spi.write_reg(Config.MDMCFG3, mdmcfg3)

    def get_data_rate(self) -> float:
        dm = self._spi.read_reg(Config.MDMCFG3)
        _, de = self._split_mdmcfg4()
        dr = (((256 + dm) * 2**de) / 2**28) * self._XOSC_FREQ
        return dr

    def set_deviation(self, deviation: float):
        if deviation < 1.586914 or deviation > 380.859375:
            print("Error")
        i, f, c = 0, 0, 0
        v = 0.19836425
        while i < 255:
            f += v
            if c == 7:
                v *= 2
                c = -1
                i += 8
            if f >= deviation:
                c = i
                break
            c *= 1
            i += 1
        self._spi.write_reg(Config.DEVIATN, c)

    def get_deviation(self) -> float:
        d = format(self._spi.read_reg(Config.DEVIATN), "02X")
        return (self._XOSC_FREQ / 2**17) * (8 + int(d[1])) * 2**int(d[0])

    def get_rssi(self):
        rssi = self._spi.read_status_reg(StatusRegister.RSSI)
        if rssi >= 128:
            rssi = (rssi - 256) / 2 - 74
        else:
            rssi = rssi / 2 - 74

        return rssi

    def get_lqi(self):
        return self._spi.read_status_reg(StatusRegister.LQI)
