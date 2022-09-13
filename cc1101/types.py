from enum import IntEnum

_CS_PINS = {
    0: {0: 8, 1: 7},
    1: {0: 18, 1: 17, 2: 16},
}


class Config(IntEnum):
    IOCFG2 = 0x00
    IOCFG1 = 0x01
    IOCFG0 = 0x02
    FIFOTHR = 0x03
    SYNC1 = 0x04
    SYNC0 = 0x05
    PKTLEN = 0x06
    PKTCTRL1 = 0x07
    PKTCTRL0 = 0x08
    ADDR = 0x09
    CHANNR = 0x0A
    FSCTRL1 = 0x0B
    FSCTRL0 = 0x0C
    FREQ2 = 0x0D
    FREQ1 = 0x0E
    FREQ0 = 0x0F
    MDMCFG4 = 0x10
    MDMCFG3 = 0x11
    MDMCFG2 = 0x12
    MDMCFG1 = 0x13
    MDMCFG0 = 0x14
    DEVIATN = 0x15
    MCSM2 = 0x16
    MCSM1 = 0x17
    MCSM0 = 0x18
    FOCCFG = 0x19
    BSCFG = 0x1A
    AGCCTRL2 = 0x1B
    AGCCTRL1 = 0x1C
    AGCCTRL0 = 0x1D
    WOREVT1 = 0x1E
    WOREVT0 = 0x1F
    WORCTRL = 0x20
    FREND1 = 0x21
    FREND0 = 0x22
    FSCAL3 = 0x23
    FSCAL2 = 0x24
    FSCAL1 = 0x25
    FSCAL0 = 0x26
    RCCTRL1 = 0x27
    FSTEST = 0x29
    PTEST = 0x2A
    AGCTEST = 0x2B
    TEST2 = 0x2C
    TEST1 = 0x2D
    TEST0 = 0x2E


class Strobe(IntEnum):
    SRES = 0x30
    SFSTXON = 0x31
    SXOFF = 0x32
    SCAL = 0x33
    SRX = 0x34
    STX = 0x35
    SIDLE = 0x36
    SAFC = 0x37
    SWOR = 0x38
    SPWD = 0x39
    SFRX = 0x3A
    SFTX = 0x3B
    SWORRST = 0x3C
    SNOP = 0x3D


class StatusRegister(IntEnum):
    PARTNUM = 0x30
    VERSION = 0x31
    FREQEST = 0x32
    LQI = 0x33
    RSSI = 0x34
    MARCSTATE = 0x35
    WORTIME1 = 0x36
    WORTIME0 = 0x37
    PKTSTATUS = 0x38
    VCO_VC_DAC = 0x39
    TXBYTES = 0x3A
    RXBYTES = 0x3B


class PTR(IntEnum):
    PATABLE = 0x3E
    TXFIFO = 0x3F
    RXFIFO = 0x3F


class Modulation(IntEnum):
    FSK2 = 0
    GFSK = 1
    ASK = 3
    FSK4 = 4
    MSK = 7


class State(IntEnum):
    IDLE = 0
    RX = 1
    TX = 2
