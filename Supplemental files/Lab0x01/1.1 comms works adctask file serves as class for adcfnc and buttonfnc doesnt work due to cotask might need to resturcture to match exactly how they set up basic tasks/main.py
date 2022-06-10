import cotask, task_share, gc, ADC_Task
from pyb import Pin, ExtInt, Timer, ADC, UART
from time import ticks_ms, ticks_diff
import array

def Button_TaskFnc():
    state = 1
    buttonFlag.put(1)
    tim6.callback(None)
    PC1.high()
    tim6.callback(tim_cb)
    
    yield state
    
# def ADC_TaskFnc():
#     state = 1
#     while True:
#         if state == 1:
#             if buttonFlag.get()==1:
                
#                 buttonFlag.put(0)
#                 start = ticks_ms()
#                 PC1.high()
#                 for i in range(m):
#                     adc_array[i] = adc.read()
#                     current = ticks_ms()
#                     time_array[i] = int(ticks_diff(current, start))
#                     print(f'{time_array[i]},{adc_array[i]}')
#                 for time_val, adc_val in zip(time_array, adc_array):
#                     uart2.write(f"{time_val}, {adc_val}\r\n")
#                     #print(f'{time_val},{adc_val}')
#                 PC1.low()
#                 collectDone.put(1)
#         else:
#             pass
#             yield state
#     else:
#         yield state
        
def tim_cb(tim6):
    state = 1
    if state == 1:
        if buttonFlag.get() == 1:
            buttonFlag.put(0)
            for time_val, adc_val in zip(time_array, adc_array):
                tim6.counter(), adc.read()
                uart2.write(f"{time_val}, {adc_val}\r\n")

                
    yield state
    
    pass
if __name__ == "__main__":
    m = 1000
    buttonFlag = task_share.Share('i', thread_protect=False, name='buttonFlag')
    collectDone = task_share.Share('i', thread_protect=False, name='collectDone')
    time_array = array.array('H', m*[0])
    adc_array = array.array('H', m*[0])
    PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP)
    PC0 = Pin(Pin.cpu.C0, mode=Pin.OUT_PP)
    adc = ADC(PC0)
    uart2 = UART(2)
    uart2.init(115200, bits=8, parity=None, stop=1)
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, None)
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback=Button_TaskFnc)
    tim6 = Timer(6, freq = 1000)
    tim6.init()
    task1 = cotask.Task (ADC_Task.ADC_TaskFnc, name = 'Task 1', priority = 1, 
                         period = 5, profile = False, trace = False)
    
    task2 = cotask.Task (ADC_Task.Button_TaskFnc, name = 'Task 2', priority = 2, 
                          period = 1, profile = False, trace = False)
    
    cotask.task_list.append (task1)
    cotask.task_list.append (task2)
    
    gc.collect ()

    
    while True: 
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break
        print('Keyboard interrupt detected terminating tasks')