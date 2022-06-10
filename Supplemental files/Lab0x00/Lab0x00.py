from pyb import Pin, ExtInt, Timer, ADC, UART
from time import ticks_us, ticks_diff, ticks_add
import array

tim6 = Timer(6, freq=1000)
PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP)
PC0 = Pin(Pin.cpu.C0, mode=Pin.OUT_PP)
          
#button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, lambda p: PC1.value(0 if PC1.value() else 1))
adc = ADC(PC0)
m = 1000
time_array = array.array('H', m*[0])
adc_array = array.array('H', m*[0])

def tim_cb(tim6):
    PC0.value(0 if PC0.value() else 1)
    PC1.high()
    tim6.init(freq=1000)
    start = tim6.counter()
    for i in range(m):
        adc_array[i] = adc.read()
        time_array[i] = (tim6.counter()-start)%100
        print(f'{time_array[i]},{adc_array[i]}')
    