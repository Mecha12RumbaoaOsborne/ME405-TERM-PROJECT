from pyb import Pin, ExtInt, Timer, ADC, UART
from time import ticks_ms, ticks_diff
import array #uart2Task

def onButtonPress(IRQ_src):
    global buttonToggle
    buttonToggle = True
    
if __name__=='__main__':
    
    #Uart object to write data to
    uart2 = UART(2)
    uart2.init(115200, bits=8, parity=None, stop=1)
    
    #
    buttonToggle = False
    tim6 = Timer(6)
    tim6.callback(None)
    m = 1500
    time_array = array.array('H', m*[0])
    adc_array = array.array('H', m*[0])
    PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP)
    PC0 = Pin(Pin.cpu.C0, mode=Pin.OUT_PP)
    adc = ADC(PC0)
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, None)
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback=onButtonPress)
    
    #When button pressed the time and adc arrays are populated using the for loop
    while True:
        try:
            if buttonToggle:
                #reset button flag
                buttonToggle=False
                start = ticks_ms()
                #toggle the input pin to high
                PC1.high()
                
                #populates adc and time arrays
                for i in range(m):
                    adc_array[i] = adc.read()
                    current = ticks_ms()
                    time_array[i] = int(ticks_diff(current, start))
                #prints data for confirmation
                for time_val, adc_val in zip(time_array, adc_array):
                    uart2.write(f"{time_val}, {adc_val}\r\n")
                #toggle back to low
                PC1.low()
                
            else:
                pass
        except KeyboardInterrupt:
            break
    print('Program Terminating')