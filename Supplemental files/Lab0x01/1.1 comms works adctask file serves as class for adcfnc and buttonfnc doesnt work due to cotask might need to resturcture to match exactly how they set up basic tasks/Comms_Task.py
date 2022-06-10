import serial, math
from matplotlib import pyplot
#Measured res and cap
# R = 102.3e3
# C = 3.24e-6
K = 3.3/4000
T = 1/1000
m=1300
raw_data = m*[0]
time_vals = m*[0]
adc_vals = m*[0]
state = 1

with serial.Serial('COM6', 115200) as uart2:
    uart2.flush()
    for idx, line in enumerate(raw_data):
        raw_data = bytes.decode(uart2.readline())
        time_vals[idx], adc_vals[idx] = raw_data.strip('\r\n').split(',')
        
    
    
    time_sec = [float(item)*T for item in time_vals]
    adc_volts = [float(item)*K for item in adc_vals]
    log_volts = [math.log((3.3-float(item))/3.3) for item in adc_volts]
    
    for time_val, adc_val in zip(time_sec, adc_volts):
        print(round(float(time_val), 3),round(float(adc_val), 3))
    
    slope = round((float(log_volts[400])-float(time_vals[0]))/(float(time_sec[400])-float(time_sec[0])), 4)    
    
    pyplot.figure(1)
    pyplot.plot(time_sec, adc_volts)
    pyplot.xlabel('Time [s]')
    pyplot.ylabel('Vout [V]')
    
    
    pyplot.figure(2)
    pyplot.plot(time_sec,log_volts)#.annotate(f'Slope:{slope}', xy = (0.25,-2.5), xytext =(0.25,-2.5))
    pyplot.xlabel('Time[s]')
    pyplot.ylabel('Log scale Voltage [-]')
    
    
    print('')
    print(f'The slope of the log plot is: {slope}')

