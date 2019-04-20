# coding: utf-8

import collections
import math
import serial
import struct
import time


# # List Important Registers
# Sourced from [datasheet](https://statics.cirrus.com/pubs/proDatasheet/CS5490_F3.pdf)

CMD_READ  = 0x00
CMD_WRITE = 0x40
CMD_PAGE  = 0x80
CMD_INST  = 0xC0

# Register 'address' includes page and nummeric format info
ADDR_MASK = 0x0000003F
PAGE_MASK = 0x00003F00
PAGE_SHFT = 8
FRAC_MASK = 0x00FF0000
FRAC_SHFT = 16
SIGN_MASK = 0x01000000
SIGN_SHFT = 24


def register(addr, page, signed=0, fracbits=0):
    return addr + (page << PAGE_SHFT) + (fracbits << FRAC_SHFT) + (signed << SIGN_SHFT)

def reg_pa(register):
    page = (register & PAGE_MASK) >> PAGE_SHFT
    addr = register & ADDR_MASK
    return page, addr

def reg_fs(register):
    frac = (register & FRAC_MASK) >> FRAC_SHFT
    sign = (register & SIGN_MASK)

# HARDWARE Page 0

CONFIG0 = register(0x00, 0, 0, 0)
CONFIG1 = register(0x01, 0, 0, 0)
MASK = register(0x03, 0, 0, 0)
PC = register(0x05, 0, 0, 0)
SERIALCTRL = register(0x07, 0, 0, 0)
PULSEWIDTH = register(0x08, 0, 0, 0)
PULSECTRL = register(0x09, 0, 0, 0)
STATUS0 = register(0x17, 0, 0, 0)
STATUS1 = register(0x18, 0, 0, 0)
STATUS2 = register(0x19, 0, 0, 0)
REGLOCK = register(0x22, 0, 0, 0)
V_PEAK = register(0x24, 0, 1, 23)  # S Q0.23
I_PEAK = register(0x25, 0, 1, 23)  # S Q0.23
PSDC = register(0x30, 0, 0, 0)
ZXNUM = register(0x37, 0, 0, 0)

# SOFTWARE Page 16
CONFIG2 = register(0x00, 16, 0, 0)
REGCHK = register(0x01, 16, 0, 0)
V_INST = register(0x02, 16, 1, 23)  # S Q0.23
I_INST = register(0x03, 16, 1, 23)  # S Q0.23
P_INST = register(0x04, 16, 1, 23)  # S Q0.23
P_AVG = register(0x05, 16, 1, 23)   # S Q0.23
I_RMS = register(0x06, 16, 0, 24)   # U Q0.24
V_RMS = register(0x07, 16, 0, 24)   # U Q0.24
Q_AVG = register(0x0E, 16, 1, 23)   # S Q0.23
Q_INST = register(0x0F, 16, 1, 23)  # S Q0.23
S = register(0x14, 16, 1, 23)       # S Q0.23 apparent power Irms * Vrms
PF = register(0x15, 16, 1, 23)      # S Q0.23
TEMP = register(0x1B, 16, 1, 16)    # S Q7.16
P_SUM = register(0x1D, 16, 1, 23)   # S Q0.23 equals P_AVG!
S_SUM = register(0x1E, 16, 1, 23)   # S Q0.23
Q_SUM = register(0x1F, 16, 1, 23)   # S Q0.23
I_DCOFF = register(0x20, 16, 1, 23)  # S Q0.23
I_GAIN = register(0x21, 16, 0, 22)  # U Q2.22
V_DCOFF = register(0x22, 16, 1, 23)  # S Q0.23
V_GAIN = register(0x23, 16, 0, 22)  # U Q2.22
P_OFF = register(0x24, 16, 1, 23)   # S Q0.23
I_ACOFF = register(0x25, 16, 1, 23)  # S Q0.23
Q_OFF = register(0x26, 16, 1, 23)   # S Q0.23
EPSILON = register(0x31, 16, 1, 23)  # S Q0.23
SAMPLECOUNT = register(0x33, 16, 0,0)
T_GAIN = register(0x36, 16, 0, 16)  # U Q8.16
T_OFF = register(0x37, 16, 1, 16)   # S Q7.16
T_SETTLE = register(0x39, 16, 0, 0)
LOAD_MIN = register(0x40, 16, 1, 23)  # S Q0.22
SYS_GAIN = register(0x3C, 16, 1, 22)  # S Q1.22
SYS_TIME = register(0x3D, 16, 0, 0)

# SOFTWARE Page 17
VSAG_DUR = register(0x00, 17, 0, 0)
VSAG_LEVEL = register(0x01, 17, 1, 23)  # S Q0.23
IOVER_DUR = register(0x04, 17, 0, 0)
IOVER_LEVEL = register(0x05, 17, 1, 23)  # S Q0.23

# SOFTWARE Page 18
IZX_LEVEL = register(0x18, 18, 1, 23)  # S Q0.23
PULSERATE = register(0x1C, 18, 1, 23)  # S Q0.23
INT_GAIN = register(0x2B, 18, 1, 23)   # S Q0.23
VSWELL_DUR = register(0x2E, 18, 0, 0)
VSWELL_LEVEL = register(0x2F, 18, 1, 23)  # S Q0.23
VZX_LEVEL = register(0x3A, 18, 1, 23)  # S Q0.23
CYCLECOUNT = register(0x3E, 18, 0, 0)
SCALE = register(0x3F, 18, 0, 0)

# Start of register metadata, will help with register dumps, value conversions, sanity check
regtable = (  # (address, string:name, int:default)
    # HW page 0
    (CONFIG0, 'config0', 0xC02000),
    (CONFIG1, 'config1', 0x00EEEE),
    (PC, 'PhaseComp', 0),
    (SERIALCTRL, 'serialctrl', 0x02004D),
    (PULSEWIDTH, 'pulsewidth', 1),
    (PULSECTRL, 'pulsectrl', 0),
    (STATUS0, 'status0', 0x800000),
    (STATUS1, 'status1', 0x801800),
    (STATUS2, 'status2', 0),
    (V_PEAK, 'v_peak', 0),
    (I_PEAK, 'i_peak', 0),

    # SW page 16
    (CONFIG2, 'config2', 0x100200),
    (REGCHK, 'regchk', 0),
    (V_INST, 'v_inst', 0),
    (I_INST, 'i_inst', 0),
    (P_INST, 'p_inst', 0),
    (P_AVG, 'p_avg', 0),
    (I_RMS, 'i_rms', 0),
    (V_RMS, 'v_rms', 0),
    (Q_AVG, 'q_avg', 0),
    (S, 's', 0),
    (PF, 'pf', 0),
    (EPSILON, 'epsilon', 0x01999A),
    (TEMP, 'temp', 0),
    (I_GAIN, 'i_gain', 0x400000),
    (V_GAIN, 'v_gain', 0x400000),
    (T_GAIN, 't_gain', 0x06B716),

    #SW page 18
    (PULSERATE, 'pulserate', 0x800000),
)

reginfo = collections.OrderedDict(((r[0], r[1:]) for r in regtable))

# Instructions (use with CMD_INST)

# Controls
SOFT_RESET = 0x1
STANDBY = 0x2
WAKEUP = 0x3
SINGLE_CONV =  0x14
CONT_CONV = 0x15
HALT_CONV = 0x18

# Calibration
I_AC_CAL = 0x31
V_AC_CAL = 0x32
IV_AC_CAL = 0x36
I_DC_CAL = 0x21
V_DC_CAL = 0x22
IV_DC_CAL = 0x26
I_GAIN_CAL = 0x39
V_GAIN_CAL = 0x3A
IV_GAIN_CAL = 0x3E


# Conversion functions between float and fixed point

def q2f(v, q, signed=True):
   '''Signed fixed point to float conversion
   val = 24 bit integer value
   q = number of fraction bits 0..23'''
   sign = v & 0x800000
   if signed:
       assert(q <= 23)
       v = v & 0x7FFFFF
       v = v - sign
   else:
       assert(q <= 24)
   return v / (1 << q)


def f2q(val, q):
   '''Float to fixed point. Works for signed and unsigned nubmers'''
   return int(0xFFFFFF & int(val * (1 << q)))

if False:
   # Quick eyeball test
   tv = (1, 0x10000, 0x7FFFFF, 0x800000, 0x810000, 0xFFFFFF)
   for v in tv:
       s = q2f(v, 23, True)
       u = q2f(v, 24, False)
       print(v, s, u, f2q(s, 23), f2q(u, 24))


class cs5490(object):
    '''Represent a CS5490 power monitor chip attached to a serial port
    '''
    def __init__(self, portname, baudrate=600, debug=False):
        '''serial is a serial.Serial object or equivalent'''
        self.current_page = None
        self.serial = serial.Serial(port=portname, baudrate=600)
        self.debug = debug
        self.reset()
        self.baud(baudrate)

    def reset(self):
        time.sleep(0.01)
        self.serial.dtr = 1  # physical low
        self.serial.baudrate = 600
        time.sleep(0.01)
        self.serial.flushInput()
        self.serial.dtr = 0  # physical high
        time.sleep(0.3)

    def baud(self, rate):
        '''Switch baud rate.  Some problems with this!'''
        if rate == self.serial.baudrate:
            return

        br =  int(round(rate * (524288 / 4096000)))
        self.write_register(SERIALCTRL, 0x020000 + br)
        time.sleep(0.1)
        self.serial.baudrate = rate

    def read_raw_register(self, register):
        '''Read 24bit register value as unsigned integer'''
        page, addr = reg_pa(register)
        if self.current_page != page:
            self.serial.write(bytearray([CMD_PAGE | page]))
            self.current_page = page
        self.serial.write(bytearray([addr]))
        buffer = bytearray(4)
        rb = self.serial.read(3)
        buffer[0:3] = rb
        return struct.unpack('<L',buffer)[0]

    def read_register(self, register):
        '''Read a value from register
        If register is defined as a fixed point number, convert it to float
        '''
        raw = self.read_raw_register(register)

        fracbits = register & FRAC_MASK
        if fracbits:  # Register is fixed point number
            fracbits >>= FRAC_SHFT
            signed = register & SIGN_MASK
            val = q2f(raw, fracbits, signed)
        else:
            val = raw

        if self.debug:
            name, default = reginfo.get(register, ('unknown', 0))
            if fracbits:
                sv = val
                dv = q2f(default, fracbits, signed)
            else:
                sv = hex(val)
                dv = hex(default)
            print('Read %10s = %06x %s [%s]' % (name, raw, sv, dv))
        return val

    def write_raw_register(self, register, val):
        '''Write 24bit register value from unsigned integer'''
        page, addr = reg_pa(register)
        if self.current_page != page:
            self.serial.write(bytearray([CMD_PAGE | page]))
            self.current_page = page
        self.serial.write(bytearray([CMD_WRITE | addr]))
        buffer = struct.pack('<L', val)
        self.serial.write(buffer[0:3])

    def write_register(self, register, val):
        '''Write converted value to register
        If register is a fixed point number, the float val will be converted
        automatically'''

        fracbits = register & FRAC_MASK
        if fracbits:  # Register is fixed point number
            fracbits >>= FRAC_SHFT
            signed = register & SIGN_MASK
            raw = f2q(val, fracbits)
        else:
            raw = val

        if self.debug:
            info = reginfo.get(register, ('%x' % register, 0))
            print('Write %10s = %s (%06X)' % (info[0], val, raw))

        self.write_raw_register(register, raw)

    def instruction(self, instr):
        self.serial.write(bytearray([CMD_INST | instr]))
        if self.debug:
            print('Instruction %x' % instr)


def main():
    portname = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A6007wZa-if00-port0'
    chip = cs5490(portname, 115200, debug=True)

    if chip.debug:
        for addr, info in reginfo.items():
            v = chip.read_register(addr)

    chip.debug = True

    chip.write_register(PULSEWIDTH, (1 << 16) | 1000) # range | width = 250us + value/64000
    chip.write_register(PULSERATE, 0.01) # rate = 2000 * value
    chip.write_register(PULSECTRL, 0)  # Pavg

    config1 = reginfo[CONFIG1][1]  # default
    config1 |= 0x110000   # Enable EPG_ON | DO_OD
    chip.write_register(CONFIG1, config1)
    time.sleep(0.12)
    config1 &= 0xFFFFF0
    config1 |= 0x000000  # DOMODE  B=V zero cross, 0=EPG
    chip.write_register(CONFIG1, config1)

    config2 =  0x100200  # default
    config2 |= 0x00000A  # Enable HPF
    #config2 |= 0x004000  # Set APCM bit
    chip.write_register(CONFIG2, config2)

    pc = 0
    #pc = 0x300000 # CPCC=3, 2 OWR delay in voltage simulate leading PF
    #pc = 0x1001FF # CPCC=1 FPCC=511, max delay in current, lagging PF
    pc = 77  # 114 = delay 1 degree in current, 1=0.008789 degrees
    chip.write_register(PC, pc)

    chip.debug = False

    # Change Voltage Gain
    #gain = 1.0
    #chip.write_register(V_GAIN, gain)

    # Experimentally determined with a known resistive load
    # and specific breadboard setup
    vscale = 563
    iscale = 44.2
    pscale = vscale * iscale

    chip.instruction(CONT_CONV)
    time.sleep(1)

    later = time.time()
    interval = 1.0

    # # Read Instantaneous voltage,current and power
    n = 10
    while n > 0:
        #n -= 1
        v = chip.read_register(V_RMS) * vscale
        i = chip.read_register(I_RMS) * iscale
        p = chip.read_register(P_AVG) * pscale  # Active power
        q = chip.read_register(Q_AVG) * pscale  # Reactive power
        # s = chip.read_register(S) * pscale  # Apparent power
        s = v * i

        pf = chip.read_register(PF)
        phase = math.degrees(math.acos(pf))
        freq = chip.read_register(EPSILON) * 4000
        t = chip.read_register(TEMP)

        print("{:.3f}Vrms  {:.3f}Arms  P={:.3f}Wavg  Q={:.3f}VARavg".format(v, i, p, q))
        print("{:.2f}Hz  {:.1f}℃   S={:.3f}VA  PF={:.3f}  Phase={:5.3f}°".format(freq, t, s, pf, phase))
        print('-' * 60)

        later += interval
        time.sleep(max(0, later - time.time()))


if __name__ == '__main__':
    main()
