from pyb import UART 
import serial, math, array
from matplotlib import pyplot



while True:
    try:
        with serial.Serial('COM3', 115200) as uart2:
            uart2.flush()
            raw_data = []
            C = 3.3/3900
            T = 1/1000
            time_vals = len(raw_data)*[0]
            adc_vals = len(raw_data)*[0]
            if uart2.any():
                uart2.readline(raw_data)
                for idx, line in enumerate(raw_data):
                    time_vals[idx], adc_vals[idx] = map(int, line.decode().strip('\r\n').split(','))
                    
                print("Raw data:", raw_data)
                print('')
                print("Parsed time data:", time_vals)
                print('')
                print("Parsed adc data",adc_vals)
                time_sec = [element*T for element in time_vals]
                adc_volts = [element*C for element in adc_vals]
                log_volts = [math.log(3.3-element/3.3) for element in adc_volts]
                pyplot.figure(1)
                pyplot.plot(time_sec, adc_volts)
                pyplot.xlabel('Time [s]')
                pyplot.ylabel('Vout [V]')
                
                pyplot.figure(2)
                pyplot.plot(time_sec,log_volts)
                pyplot.xlabel('Time[s]')
                pyplot.ylabel('Log scale Voltage [-]')
                
            else:
                pass
    except KeyboardInterrupt:
        break
    print('Program Terminating')