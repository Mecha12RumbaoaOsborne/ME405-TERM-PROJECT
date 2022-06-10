"""!@file TMC4210.py
@brief      Class file for stepper drivers.
@n
@page driver  TMC4210.py
@brief      This class file contains the methods and attributes of the TMC4210 drivers.
@details    The file includes methods for setting target position/velocity, reading 
            position/velocity, writing to configuration adresses, and homing/zeroing
            the stepper motors.. 

Source Folder
    [https://github.com/Mecha12RumbaoaOsborne/ME405-TERM-PROJECT]
@n

@author Barret Osborne
@author Ruodolf Rumbaoa
@date 06/09/2022

"""
from pyb import SPI, Pin, Timer
import time

# Stepper Register Set
#X_TARGET and V_TARGET (1 byte size) are made as bytes to be concatenated
#with integer values that the user inputs which are then turned to
#bytes objects (3 bytes size) to create a proper 32 bit datagram.
#Other adresses are hex values inputed into a bytearray with specific
#binary data to configure devices. SEE  TMC4210 data sheet for register info.

X_TARGET             = (0x00).to_bytes(1, 'big')
X_ACTUAL             = 0x02
V_MIN                = 0x04
V_MAX                = 0x06
V_TARGET             = (0x08).to_bytes(1, 'big')
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

# Global Parameter regs
IF_CONFIGURATION_4210= 0x68
POS_COMP_4210        = 0x6A
POS_COMP_INT_4210    = 0x6C
POWER_DOWN           = 0x70
TYPE_VERSION         = 0x73
REFERENCE_SWITCHES   = 0x7C
GLOBAL_PARAMETERS    = 0x7E
# R = 1 W = 0

class TMC4210drv:
    
    def __init__(self, spi_num, spi_mode, spi_baud, spi_phase, spi_pol, spi_crc, ncsPin, clk_timer, clk_channel, clkPin, ENPin):
        '''
        @brief      Initializes the drviver objects.
        @details    The driver objects are initialized as SPI2 peripherals with 
                    pin objects for clock timer/channel and enable PIN.
        @param spi_num SPI bus to initialize the driver (using SPI2)
        @param spi_mode SPI.PERIPHERAL
        @param spi_baud Baudrate for SPI transmission (using 1_000_000).
        @param spi_phase spi_pol These parameters dictate the mode being ran (using mode 3)
        @param spi_crc None
        @param ncsPin Pin object used for chip select
        @param clk_timer Timer object being used by drivers
        @param clk_channel Timer channel
        @param clkPin Pin object being used for clock signal 
        @param ENPin Pin object used for EN
        '''
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
        '''
        @brief enables the stepper
        '''
        #Sets en_sd bit to 1
        enableba = bytearray([IF_CONFIGURATION_4210, 0x00, 0x00, 0x20])
        
        #Set enable pin low to activate power to motor
        self.EN.low()
        
        self.nCS.low()
        self.spi.send_recv(enableba, timeout=2000)
        self.nCS.high()
        print('The motor is enabled.')
        
        pass
    
    def setRampmode(self):
        '''
        @brief sets the driver to ramp mode. used for position control
        '''
        #Sets the driver to Ramp mode. Used for position control
        rampmode = bytearray([0x14, 0x00, 0x00, 0b00000000])
        
        self.nCS.low()
        self.spi.send_recv(rampmode, timeout=2000)
        self.nCS.high()
        
        pass
    
    def setVelmode(self):
        '''
        @brief sets the drivers to velocity mode. used for constant velocity control
        '''
        #Sets the driver to Velocity mode. Used for constant velocity applications
        velmode = bytearray([0x14, 0x00, 0x00, 0b00000010])
        
        self.nCS.low()
        self.spi.send_recv(velmode, timeout=2000)
        self.nCS.high()
        
        pass
    
    def setHoldmode(self):
        '''
        @breif sets the drvier to hold mode. used to overwrite actual position
               when writing to X_ACTUAL address
        '''
        #Sets the driver to Velocity mode. Used for constant velocity applications
        holdmode = bytearray([0x14, 0x00, 0x00, 0b00000011])
        
        self.nCS.low()
        self.spi.send_recv(holdmode, timeout=2000)
        self.nCS.high()
        
        pass
    def disable(self):
        '''
        @breif disables the motors
        '''
        #Set enable pin high to disable motor power
        self.EN.high()
        
        pass
    
    def getPos(self):
        '''
        @breif reads the position of the motor in microsteps
        '''
        #Read from X_ACTUAL address
        getPos = bytearray([0x03, 0x00, 0x00,0x00])
        
        self.nCS.low()
        self.data = self.spi.send_recv(getPos)
        self.nCS.high()
        
        self.intVal = int.from_bytes(self.data, 'big')-352321536
        
        return self.intVal
    
    def setPos(self, inputPos):
        '''
        @breif sets target position of stepper
        '''
        #Convert integer input to a bytes objects
        inputPos = (inputPos).to_bytes(3, 'big')

        #Write to X_TARGET 
        setPos = X_TARGET+inputPos
        
        self.nCS.low()
        self.data = self.spi.send_recv(setPos)
        self.nCS.high()
        
        pass
    
    def goZero(self):
        '''
        @breif writes X_TARGET to go to zero defined in beginning of main routine
        '''
        #Write to X_TARGET. Goes back to 0
        gozero = X_TARGET+bytearray([0b00000000, 0b00000000, 0b00000000])
        
        self.nCS.low()
        self.spi.send_recv(gozero)
        self.nCS.high()
        
        pass
    
    def zeroHere(self):
        '''
        @breif defines the zero position of stepper
        '''
        setZero = bytearray([X_ACTUAL, 0b00000000,0b00000000, 0b00000000])
        
        #Need to set to Velocity mode before overwrting X_ACTUAL
        self.setHoldmode()
        self.goZero()

        self.nCS.low()
        self.spi.send_recv(setZero)
        self.nCS.high()
        
        self.setRampmode()
        
        print(f'The position has been set to 0 [microsteps]')
        
        pass
    
    def setVelT(self, inputVel):
        '''
        @breif sets target velocity when in velocity mode
        '''
        #Write to V_TARGET 
        setVelT = V_TARGET + (0x00).to_bytes(2, 'big') + (inputVel).to_bytes(1, 'big', signed=True)

        self.setVelmode()
        
        self.nCS.low()
        self.spi.send_recv(setVelT)
        self.nCS.high()
        
        pass
    
    def getVel(self):
        '''
        @breif reads velocity of stepper
        '''
        #Read from V_ACTUAL address
        getVel = bytearray([V_ACTUAL, 0x00, 0x00,0x00])
        
        self.nCS.low()
        self.data = self.spi.send_recv(getVel)
        self.nCS.high()
        
        for idx,byte in enumerate(self.data): print(f"b{3-idx}: {byte:#010b} {byte:#04x}")
        
        pass
    
    def setMaxVel(self, inputMaxvel):
        '''
        @breif configures max velocity of stepper. recommend to use low speeds
               for position control
        '''
        #Write to V_MAX address
        setMaxvel = bytearray([V_MAX,0b00000000, 0b00000, inputMaxvel])
        
        self.nCS.low()
        self.spi.send_recv(setMaxvel)
        self.nCS.high()
        
        print(f'The max velocity has been set to {inputMaxvel} []')
        pass
    
    def setMinVel(self, inputMinvel):
        '''
        @breif configures min velocity of stepper. recommend to use low speeds
               for position control
        '''
        #Write to V_MIN address
        setMinvel = bytearray([V_MIN, 0b00000000, 0b00000, inputMinvel])
        
        self.nCS.low()
        self.spi.send_recv(setMinvel)
        self.nCS.high()
        
        print(f'The min velocity has been set to {inputMinvel} []')
        
        pass
    
    
    def setMaxAcc(self, inputMaxacc1, inputMaxacc2):
        '''
        @breif configures max acceleration of stepper
        '''
        #Write to A_MAX address
        setMaxacc = bytearray([A_MAX, 0b00000000, inputMaxacc1, inputMaxacc2])

        self.nCS.low()
        self.data = self.spi.send_recv(setMaxacc)
        self.nCS.high()

        print(f'The max accelertion has been set to []')
        
        pass
    
    def setPulseramppulsediv(self, inputPramp, inputPlsdiv):
        '''
        @breif configures pulse ramp and pulse divider for stepper frequencies
        '''
        #Write to PULSE_DIV_RAMP_DIV address
        setPramppdiv = bytearray([PULSE_DIV_RAMP_DIV, 0b00000000, inputPramp, inputPlsdiv, 0b00000000])
        
        self.nCS.low()
        self.data = self.spi.send_recv(setPramppdiv)
        self.nCS.high()

        print(f'The PULSE_RAMP has been set to {inputPramp}')
        print(f'The PULSE_DIV has been set to {inputPlsdiv}')
        
        pass
    
    def setPMULPDIV(self, inputPMUL, inputPDIV):
        '''
        @breif configures PMUL and PDIV. need to find a matching pair according
               according to datasheet
        '''
        #Write to PMUL_PDIV
        setPMULPDIV = bytearray([PMUL_PDIV, 0b00000001, inputPMUL, inputPDIV])
        
        self.nCS.low()
        self.data = self.spi.send_recv(setPMULPDIV)
        self.nCS.high()

        print(f'The PMUL has been set to {inputPMUL}')
        print(f'The PDIV has been set to {inputPDIV}')

        pass
    
if __name__ =='__main__':
    drv1 = TMC4210drv(2, SPI.SLAVE, 1_000_000, 1, 1, None, Pin(Pin.cpu.C3, mode=Pin.OUT_PP, value =1), 4, 1, Pin(Pin.cpu.B6, mode = Pin.OUT_PP), Pin(Pin.cpu.C2, mode=Pin.OUT_PP, value =1))
    drv2 = TMC4210drv(2, SPI.SLAVE, 1_000_000, 1, 1, None, Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value =1), 4, 1, Pin(Pin.cpu.B7, mode = Pin.OUT_PP), Pin(Pin.cpu.C0, mode=Pin.OUT_PP, value =1))
    
    uC = SPI(2)
    uC.init(SPI.MASTER, baudrate=1_000_000, phase=1, polarity=1, crc=None)
    
    #Set ensd to 1 for both drivers
    drv1.enable()
    drv2.enable()
   
    drv1.setRampmode()
    drv2.setRampmode()
    
    #Set VMIN
    drv1.setMinVel(0b00000000010)
    drv2.setMinVel(0b00000000010)
    
    #Set VMAX
    drv1.setMaxVel(0b00000000010)
    drv2.setMaxVel(0b00000000010)
    
    #Set Pulse div and ramp div
    drv1.setPulseramppulsediv(0b0000, 0b0000)
    drv2.setPulseramppulsediv(0b0000, 0b0000)
    
    #Set AMAX
    drv1.setMaxAcc(0b00000111,0b11010000)
    drv2.setMaxAcc(0b00000111,0b11010000)

    #Set PDIV 1 and PMUL 249
    drv1.setPMULPDIV(0b11111001, 0b0000001)
    drv2.setPMULPDIV(0b11111001, 0b0000001)
    
 