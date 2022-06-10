import cotask, task_share, gc, array
from pyb import Pin, ExtInt, Timer, ADC, UART

def adcTask():
    while True:
        if buttonFlag.get() == 1:
            adcData.append(adc.read())
            timeData.append(tim6.counter())
            fullData.put(f'{tim6.counter()},{adc.read()}\r\n')
        else:
            state = 0
            pass
    yield state
        
def onButtonpress(IRQ_src):
    buttonFlag.put(1)
    state = 1
    
if __name__ == "__main__":
    print ('This program will automatically run a step response.\r\n'
           'Once enough data is collected it will send the data to be plotted\r\n'
           'and wait for another button press to collect data again')
    PC1 = Pin(Pin.cpu.C1, mode=Pin.OUT_PP)
    PC0 = Pin(Pin.cpu.C0, mode=Pin.OUT_PP)
    adc = ADC(PC0)
    adcData = []
    timeData = []
    
    uart2 = UART(2)
    uart2.init(115200, bits=8, parity=None, stop=1)
    
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, None)
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback=onButtonpress)
    
    tim6 = Timer(6)
    
    buttonFlag = task_share.Share ('i', thread_protect = False, name = "buttoFlag")
    fullData = task_share.Queue ('B', 8, thread_protect = True, overwrite = False,
                           name = "fullData")

    adcTask = cotask.Task (adcTask, name = 'Task_1', priority = 1, 
                         period = .001, profile = True, trace = False)
    # task2 = cotask.Task (task2_fun, name = 'Task_2', priority = 2, 
    #                      period = 1500, profile = True, trace = False)
    
    cotask.task_list.append (adcTask)
    # cotask.task_list.append (task2)
    gc.collect ()

    # vcp = pyb.USB_VCP ()
    # while not (vcp.any ()) or (len(adcData) == 1300):
    #     cotask.task_list.pri_sched ()
    # vcp.read ()
    tim6.init(freq=1000)
    
    while not fullData.num_in() == 1300:
        cotask.task_list.pri_sched ()
    state = 0
    buttonFlag.put(0)
    
    for time_val, adc_val in zip(timeData, adcData):
        #fullData.put(f"{time_val}, {adc_val}\r\n")
        uart2.write(f"{time_val}, {adc_val}\r\n")
    