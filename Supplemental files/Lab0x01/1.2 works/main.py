import cotask, task_share, gc, array
from pyb import Pin, ExtInt, Timer, ADC, UART
import pyb
from time import ticks_ms, ticks_diff

def adcTask():
    while True:
        if buttonFlag.get() == 1:
            timeData.put(int(ticks_diff(ticks_ms(), start)))
            adcData.put(int(adc.read()))
            uart2.write(f'{timeData.get()}, {adcData.get()}\r\n')
        else:
            pass
    yield state
        
def onButtonpress(IRQ_src):
    buttonFlag.put(1)
    PC1.high()
    
if __name__ == "__main__":
    print ('This program will automatically run a step response.\r\n'
           'Once enough data is collected it will send the data to be plotted\r\n'
           'and wait for another button press to collect data again')
    
    PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP)
    PC0 = Pin(Pin.cpu.C0, mode=Pin.OUT_PP)
    adc = ADC(PC0)

    uart2 = UART(2)
    uart2.init(115200, bits=8, parity=None, stop=1)
    
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, None)
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback=onButtonpress)
    
    tim6 = Timer(6)
    
    #Flag and two shares for button press, time vals and adc vals
    buttonFlag = task_share.Share ('i', thread_protect = False, name = "buttonFlag")
    timeData = task_share.Queue ('I', 1300, thread_protect = True, overwrite = False,
                           name = "timeData")
    adcData = task_share.Queue ('I', 1300, thread_protect = True, overwrite = False,
                           name = "adcData")
    #adc task being ran at 
    adcTask = cotask.Task (adcTask, name = 'Task_1', priority = 1, 
                         period = 1000, profile = True, trace = False)

    cotask.task_list.append (adcTask)

    gc.collect ()
    tim6.counter(0)
    tim6.init(freq=1000)
    
    timeData.clear()
    adcData.clear()
    PC1.low()
    
    start = ticks_ms()
    state = 1
    
    while True:
        
        if not adcData.full():
            cotask.task_list.pri_sched()
        else:
            pass
        
    PC1.low()
    tim6.deinit()
    buttonFlag.put(0)
    state = 2
        

        
        

