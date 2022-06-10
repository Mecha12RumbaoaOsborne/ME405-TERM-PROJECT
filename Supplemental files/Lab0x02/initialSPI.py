from pyb import SPI, Pin, Timer
#Stepper Register Set
X_TARGET             = 0x00
X_ACTUAL             = 0x02
V_MIN                = 0x04
V_MAX                = 0x06
V_TARGET             = 0x08
V_ACTUAL             = 0x0A
A_MAX                = 0x0C
A_ACTUAL             = 0x0E
PMUL_PDIV            = 0x12
RAMPMODE_REFCONF     = 0x14
INTERRUPT_MASK_FLAGS = 0x16
PULSE_DIV_RAMP_DIV   = 0x18
DX_REF_TOLERANCE     = 0x1A
X_LATCHED            = 0x1C
USTEP_COUNT_4210     = 0x1E

#Global Parameter regs
IF_CONFIGURATION_4210= 0x68
POS_COMP_4210        = 0x6A
POS_COMP_INT_4210    = 0x6C
POWER_DOWN           = 0x70
TYPE_VERSION         = 0x72
REFERENCE_SWITCHES   = 0x7C
GLOBAL_PARAMETERS    = 0x7E
spi = SPI(2)

PC3 = Pin(Pin.cpu.C3, mode = Pin.OUT_PP)
PB0 = Pin(Pin.cpu.B0, mode = Pin.OUT_PP)
nCS1 = Pin(Pin.cpu.C3, mode=Pin.OUT_PP, value =1)
nCS2 = Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value =1)

tim4 = Timer(4)
tim4.init(prescaler = 0, period =3)
PB6 = Pin(Pin.cpu.B6, mode = Pin.OUT_PP)
en1 = Pin(Pin.cpu.C2, mode=Pin.OUT_PP)


clk = tim4.channel(1, mode=Timer.PWM, pin = PB6, pulse_width = 2)

spi.init(SPI.MASTER, baudrate = 1000000, polarity = 1, phase = 1, crc=None)

# ba = bytearray([0x73,
#               0b00000000,
#               0b00000000,
#               0b00000000])

#Set EN pin low
en1.low()

#Set en_sd to 1
ba = bytearray([0x73, 0b00000000, 0b00000000, 0b000000])
#ba = bytearray([0x01, 0b00000000, 0b00000000, 0b0000000])

en1.low()

nCS1.low()
data = spi.send_recv(ba, timeout = 5000)
nCS1.high()

for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
# 
# #setting up min max vals
# minvel = bytearray([0x02, 0x00, 0b00000, 0b00000000000])
# maxvel = bytearray([0x04, 0x00, 0b00000, 0b11111111111])
# 
# nCS2.low()
# data = spi.send_recv(minvel, timeout = 5000)
# nCS2.high()
# nCS2.low()
# data = spi.send_recv(maxvel, timeout = 5000)
# nCS2.high()
# 
# #set puls div and ramp div
# pulseramp = bytearray([0b00011000, 0b00000000, 0b00000000, 0b00000000])
# 
# nCS2.low()
# data = spi.send_recv(pulseramp, timeout = 5000)
# nCS2.high()
# 
# #set max acc
# maxacc = bytearray([0b00001100,0b00000000,0b00000111,0b11010000])
# 
# nCS2.low()
# data = spi.send_recv(maxacc, timeout = 5000)
# nCS2.high()
# 
# #set pmul 249 pdiv 1
# pmuldiv = bytearray([0b00010010, 0b00000000, 0b11111001, 0b0000001])
# 
# #set ramp mode
# rampmode = bytearray([0x14, 0x00, 0x00, 0x00])
# 
# nCS2.low()
# data = spi.send_recv(rampmode, timeout = 5000)
# nCS2.high()
# 
# #zero before writing to xtarget
# velocitymode = bytearray([0b00001000, 0b00000000, 0b00000000,0b01000000])
# 
# nCS2.low()
# data = spi.send_recv(velocitymode, timeout = 5000)
# nCS2.high()
# 
# zeropos = bytearray([0b00000000, 0b00000000, 0b00000000, 0b00000000])
# nCS2.low()
# data = spi.send_recv(rampmode, timeout = 5000)
# nCS2.high()
# 
# #write to xtarget
# xtarget = bytearray([0b00000000, 0b11000011,0b01010000,0b000000])
# 
# nCS2.low()
# data = spi.send_recv(xtarget, timeout = 5000)
# nCS2.high()
# nCS2.low()
# spi.send_recv(minvel, recv=recieve, timeout = 1000)
# spi.send_recv(maxvel, recv=recieve, timeout = 1000)
# spi.send_recv(pulseramp, recv=recieve, timeout = 1000)
# spi.send_recv(accmax, recv=recieve, timeout = 1000)
# spi.send_recv(pmuldiv, recv=recieve, timeout = 1000)
# spi.send_recv(vtarget, recv=recieve, timeout = 1000)
# nCS2.high()
# 
