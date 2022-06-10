from pyb import SPI, Pin, Timer
import time
# Stepper Register Set
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
# INTERRUPT_MASK_FLAGS = 0x16
PULSE_DIV_RAMP_DIV   = 0x18
# DX_REF_TOLERANCE     = 0x1A
# X_LATCHED            = 0x1C
# USTEP_COUNT_4210     = 0x1E

# Global Parameter regs
IF_CONFIGURATION_4210= 0x68
# POS_COMP_4210        = 0x6A
# POS_COMP_INT_4210    = 0x6C
POWER_DOWN           = 0x70
TYPE_VERSION         = 0x73
REFERENCE_SWITCHES   = 0x7C
# GLOBAL_PARAMETERS    = 0x7E
# R = 1 W = 0

class TMC4210:
    
    def __init__(self, spi_num, spi_mode, spi_baud, spi_phase, spi_pol, spi_crc, ncsPin, clk_timer, clk_channel, clkPin, ENPin):

        #use spi(2)
        self.spi = SPI(spi_num)
        
        #use params: SPI.SLAVE, 1000000, 1, 1, None 
        self.spi.init(spi_mode, baudrate=spi_baud, phase=spi_phase, polarity=spi_pol, crc=spi_crc)
        
        #use C3 or B0 for CS pins
        self.nCS = Pin(ncsPin, mode = Pin.OUT_PP, value = 1)
        
        #use C2 or C0 for EN pins
        self.EN = Pin(ENPin, mode = Pin.OUT_PP, value = 0)
        
        #use Timer(4)
        self.clk = Timer(clk_timer)
        
        #20 mHz timer
        self.clk.init(prescaler = 0, period =3)
        
        # need PWM mode pin PB6 for clkPin
        self.clk.channel(clk_channel, mode=Timer.PWM, pin = clkPin, pulse_width = 2)
        
        pass
       
    def enable(self):
        
        #Sets en_sd bit to 1
        enableba = bytearray([IF_CONFIGURATION_4210, 0x00, 0x00, 0x20])
        
        self.EN.low()
        self.nCS.low()
        #Stores buffer recieved back from driver
        self.spi.send_recv(enableba, timeout=2000)
        self.nCS.high()
        print('The motor is enabled.')
        
        pass
    
    def setRamp(self):
        rampmode = bytearray([0x14, 0x00, 0x00, 0x00])
        
        self.nCS.low()
        self.spi.send_recv(rampmode, timeout=2000)
        self.nCS.high()
        
        
        #checks what is recieved back from driver
        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        
        pass
    
    def disable(self):
        self.EN.high()
        pass
    
    def getPos(self):
        
        #Read from X_ACTUAL address
        getPos = bytearray([X_ACTUAL, 0x00, 0x00,0x00])
        
        self.nCS.low()
        self.spi.send_recv(getPos)
        self.nCS.high()

        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        #intdata = int.from_bytes(data, 'big', signed = True)
        
        #print(f'The motor is currently at: {intdata} [microsteps]')
    
    def setPos(self, inputPos1, inputPos2, inputPos3):
        #inputPos ranges from 0 to 
        #inputPos1 = inputPos.to_bytes(24, 'big')
        #inputPos = int(bin(inputPos))
        #inputPos1 = hex(inputPos).strip('')
        
        setPos = bytearray([X_TARGET, inputPos1, inputPos2, inputPos3])
        #setPos = bin(int.from_bytes(setPos, 'big', signed = True))


        self.nCS.low()
        self.spi.send_recv(setPos)
        self.nCS.high()

        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        #print(f'The position has been set to {inputPos} [microsteps]')
        
        pass
    
    def zero(self):
        #inputPos ranges from 0 to 
        #inputPos = (inputPos).to_bytes(24, 'big')
        #inputPos = int(bin(inputPos))
        
        setPos = bytearray([X_ACTUAL, 0b00000000,0b00000000, 0b00000000])
        #setPos = bin(int.from_bytes(setPos, 'big', signed = True))
        setHold = bytearray([RAMPMODE_REFCONF, 0b00000000, 0b00000000, 0b00000011])
        setRamp = bytearray([RAMPMODE_REFCONF, 0b000000000, 0b0000000,0b00000000])
        
        self.nCS.low()
        self.spi.send_recv(setHold)
        self.nCS.high()

        self.nCS.low()
        self.spi.send_recv(setPos)
        self.nCS.high()
        
        self.nCS.low()
        self.spi.send_recv(setRamp)
        self.nCS.high()

        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        #print(f'The position has been set to {inputPos} [microsteps]')
        
        pass
    
    def getVel(self):
        #Read from V_ACTUAL address
        getVel = bytearray([V_ACTUAL, 0x00, 0x00,0x00])
        
        self.nCS.low()
        data = self.spi.send_recv(getVel)
        self.nCS.high()

        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        #intdata = int.from_bytes(data, 'big', signed = True)
        
        #print(f'The motor is currently spinning at: {intdata} [rpm]')
        pass
    
    def setMaxVel(self, inputMaxvel):
        #(inputMaxvel).to_bytes(11, 'big')
        #inputMaxvel = bin(inputMaxvel)
        setMaxvel = bytearray([V_MAX,0b00000000, 0b00000, inputMaxvel])
        #setMaxvel = bin(int.from_bytes(setMaxvel, 'big', signed = True))


        self.nCS.low()
        self.spi.send_recv(setMaxvel)
        self.nCS.high()
        
        self.nCS.low()
        check = self.spi.send_recv(bytearray([0x07, 0x00, 0x00, 0x00]))
        self.nCS.high()

        for idx,byte in enumerate(check): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        print(f'The max velocity has been set to {inputMaxvel} []')
        pass
    
    def setMinVel(self, inputMinvel):
        #(inputMinvel).to_bytes(11, 'big')
        #inputMinvel = bin(inputMinvel)
        setMinvel = bytearray([V_MIN, 0b00000000, 0b00000, inputMinvel])
        #setMinvel = bin(int.from_bytes(setMinvel, 'big', signed = True))


        self.nCS.low()
        self.spi.send_recv(setMinvel)
        self.nCS.high()
        
        self.nCS.low()
        check = self.spi.send_recv(bytearray([0x05, 0x00, 0x00, 0x00]))
        self.nCS.high()
        
        for idx,byte in enumerate(check): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        print(f'The min velocity has been set to {inputMinvel} []')
        
        pass
    
    def getAcc(self):
        pass
    
    def setMaxAcc(self, inputMaxacc1, inputMaxacc2):
        #(inputMaxacc).to_bytes(11, 'big')
        #inputMaxacc = bin(inputMaxacc)
        
        setMaxacc = bytearray([A_MAX, 0b00000000, inputMaxacc1, inputMaxacc2])
        #setMaxacc = bin(int.from_bytes(setMaxacc, 'big', signed = True))

        self.nCS.low()
        data = self.spi.send_recv(setMaxacc)
        self.nCS.high()

        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        self.nCS.low()
        check = self.spi.send_recv(bytearray([0x0D, 0x00, 0x00, 0x00]))
        self.nCS.high()

        for idx,byte in enumerate(check): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        print(f'The max accelertion has been set to []')
        pass
    
    def setPulseramppulsediv(self, inputPramp, inputPlsdiv):
        #(inputPramp).to_bytes(4, 'big')
        #(inputPlsdiv).to_bytes(4, 'big')
        #inputPramp = bin(inputPramp)
        #inputPlsdiv = bin(inputPlsdiv)
        setPramppdiv = bytearray([PULSE_DIV_RAMP_DIV, 0b00000000, inputPramp, inputPlsdiv, 0b00000000])
        #setPramppdiv = bin(int.from_bytes(setPramppdiv, 'big', signed = True))


        self.nCS.low()
        data = self.spi.send_recv(setPramppdiv)
        self.nCS.high()

        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        print(f'The PULSE_RAMP has been set to {inputPramp}')
        print(f'The PULSE_DIV has been set to {inputPlsdiv}')
        pass
    
    def setPMULPDIV(self, inputPMUL, inputPDIV):
        #(inputPMUL).to_bytes(7, 'big')
        #(inputPDIV).to_bytes(4, 'big')
        #inputPMUL = bin(inputPMUL)
        #inputPDIV = bin(inputPDIV)
        setPMULPDIV = bytearray([PMUL_PDIV, 0b00000001, inputPMUL, inputPDIV])
        #setPMULPDIV = bin(int.from_bytes(setPMULPDIV, 'big', signed = True))

        self.nCS.low()
        data = self.spi.send_recv(setPMULPDIV)
        self.nCS.high()

        #for idx,byte in enumerate(data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        print(f'The PMUL has been set to {inputPMUL}')
        print(f'The PDIV has been set to {inputPDIV}')

        pass
    
if __name__ =='__main__':
#     drv1 = TMC4210(2, SPI.SLAVE, 1_000_000, 1, 1, None, Pin(Pin.cpu.C3, mode=Pin.OUT_PP, value =1), 4, 1, Pin(Pin.cpu.B6, mode = Pin.OUT_PP))
#     drv2 = TMC4210(2, SPI.SLAVE, 1_000_000, 1, 1, None, Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value =1), 4, 2, Pin(Pin.cpu.B7, mode = Pin.OUT_PP))
    uC = TMC4210(2, SPI.MASTER, 1_000_000, 1, 1, None, Pin(Pin.cpu.C3, mode=Pin.OUT_PP, value = 1), 4, 1, Pin(Pin.cpu.B6, mode = Pin.OUT_PP),
                 Pin(Pin.cpu.C2, mode = Pin.OUT_PP, value=1))
    #Set ensd to 1 for both drivers
#     drv1.enable()
#     drv2.enable()
    uC.enable()
   
    uC.setRamp()
    #Set VMIN, VMAX
    uC.setMinVel(0b00000000100)
#     drv2.setMinVel(0)
    uC.setMaxVel(0b00000001000)
#     drv2.setMaxVel(10000)
    
    #Set Pulse div and ramp div
    uC.setPulseramppulsediv(0b0000, 0b0000)
    
    #Set AMAX
    uC.setMaxAcc(0b00000111,0b11010000)
#     drv2.setMaxAcc(2000)

    #Set PDIV 1 and PMUL 249
    uC.setPMULPDIV(0b11111001, 0b0000001)
#     drv2.setPMULPDIV(249, 1)
    
    #Zero before running and set to rampmode
    
    
    #uC.zero()
    #Set pos
    uC.setPos(0b00000000, 0b0000100, 0b0000000)
    
    
    
        