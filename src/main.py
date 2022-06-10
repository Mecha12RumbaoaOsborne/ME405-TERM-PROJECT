"""!@file main.py
@brief      Main file for polar plotter project.
@n
@page mainroutine  main.py
@brief      This main routine makes use of the provided scheduler to run the parsing
            of an hpgl file saved in baord memory, and sending insturctions(pen up/down)
            and coordinates to the plotter.
@details    When ran, the main routine waits for the user to press the blue button on the 
            NucleoL476RG board. Once this press is detected, the scheduler runs the parsing
            and drawing task to extract instructions and coordinates from the hpgl file.
            The instructions are interpreted as pen up/down and the DC motor actuates
            according to it. The xy coordinates are converted to theta 1 and theta 2 values
            sento to the repective stepper. Once the task finishes a flag is raised and 
            the steppers go back to home position. 
@n
@image html name.png

@n     
Source Folder
    [https://github.com/Mecha12RumbaoaOsborne/ME405-TERM-PROJECT]
@n

@author Barret Osborne
@author Ruodolf Rumbaoa
@date 06/09/2022

"""
from ulab import numpy as np
#import numpy as np
import cotask, task_share, gc, array
from time import sleep
from math import cos, sin, pi, tan, sqrt, acos, asin, atan2
from pyb import SPI, Pin, Timer, ExtInt, UART
from TMC4210 import TMC4210drv

def onButtonpress(IRQ_src):
    '''
    @breif Button flag used to start the drawing task
    '''
    buttonFlag.put(1)
    
def g(x, theta, r):
    '''
    @breif Calculates motor thetas from xy coordinate inputs
    '''
    g1 = x[0]-r*theta[1]*float(cos(theta[0])) 
    g2 = x[1]-r*theta[1]*float(sin(theta[0]))
    return np.array([g1, g2])

def dg_dtheta(theta, r):
    '''
    @breif Calculates the jacobian
    '''
    return np.array([[r*theta[1]*float(sin(theta[0])), -r*float(cos(theta[0]))],
                            [-r*theta[1]*float(cos(theta[0])), -r*float(sin(theta[0]))]]) 

def NewtonRaphson (fcn, jacobian, guess, thresh):
    '''
    @breif Calculates the new thetas approximated by NewtonRaphson method
    '''
    new_theta = guess - (np.dot(np.linalg.inv(jacobian),fcn))
    return new_theta

def iterate(fcn, x, theta, r, thresh):
    '''
    @breif Used to iteratively call NewtoRaphson method to find motor thetas
    '''
    while True:
        newTheta = fcn(g(x, theta, r), dg_dtheta(theta, r), theta, thresh)
        gtheta = g(x, theta, r)
    
        if np.linalg.norm(gtheta) > thresh:
            theta = newTheta
            continue
        else:
            correct_theta = newTheta
            break
    return correct_theta

def parse(file):
    '''
    @breif Opens the hpgl file defined and extracts pen up/down instructions
           and xy coordinates of each point
    @param file The hpgl file to be prsed
    '''
    with open(file, 'r') as f:
        for item in f:
            command = item.split(';')
        for i in command:
            f2 = i[0:2]
            if f2 == 'PU':
                if len(i) > 2:
                    shapes.append(extract(i))
            elif f2 == 'PD':
                if len(i) > 2:
                    shapes.append(extract(i))
            else:
                pass
            f.close() 
            shapesInst = shapes
    return shapesInst

def extract(insts):
    '''
    @breif Splits extracted instructions by commas to generate xy coordinates
    @param insts Instructions extracted from parsed file
    '''
    instOut = []
    val = insts.split(',')

    for n in range(len(val)>>1):    
       instOut.append([val[2*n], val[2*n+1]])
       

    return instOut

def drawTask ():
    '''
    @breif Calls the draw function and sets flag to represent when drawing is finished
    '''
    while True:
        print('DRAWING')
        xcoordData.clear()
        ycoordData.clear()
        instructions = parse(file)
        for point in instructions:
            draw(point) 
        drawingDone.put(1)
        buttonFlag.put(0)
        state == 0
        break
    yield state
        
def draw(instVal):
    '''
    @breif Uses the extracted xy coordinates to calculate motor thetas. Then sends
           motor thetas to respective stepper. Also interprets pen instructions to 
           actuate DC motor.
    '''
    inst = instVal[0]
    k=0
    if inst[0][0:2] == 'PU':
        print('pen up')
        PB5.low()
        PB4.high()
        sleep(.8)
        
        
        instVal[0][0] = instVal[0][0].strip('PU')
        
        #Construct as a share later 0 = up 1 = down 
        penShare = 0
        
        while k < len(instVal):
            incoordx = 1.5*((int(instVal[k][0])+100)/dpi)
            incoordy = 1*((int(instVal[k][1])+1)/dpi)
            xcoordData.put(incoordx)
            ycoordData.put(incoordy)
            uart2.write(f'{xcoordData.get()}, {ycoordData.get()}\r\n')
            hyp = sqrt(incoordx**2 + incoordy**2)
            
            thetas = np.array([1*(atan2(incoordy,incoordx)), hyp/inrad])
            
            usteps1 = int(thetas[0]*3600/(2*pi))
            usteps2 = int(thetas[1]*400/(2*pi))
            print(f'[{usteps1}, {usteps2}]')
            
            while True:
                stepthresh = 1
                drv1.setPos(usteps1)
                drv2.setPos(usteps2)
                sleep(.25)
                comp1 = usteps1
                comp2 = usteps2
                act1 = drv1.getPos()
                act2 = drv2.getPos()
                
                if (abs(comp1 - act1) < stepthresh) and (abs(comp2 - act2 < stepthresh)):
                    break

                else:
                    pass
                    
            k+=1
            continue
            
        
    elif inst[0][0:2] == 'PD':
        print('pen down')
        PB4.low()
        PB5.high()
        instVal[0][0] = instVal[0][0].strip('PD')
        
        #Construct as a share later 0 = up 1 = down 
        penShare = 1
        
        while k < len(instVal):
            incoordx = 1.5*((int(instVal[k][0])+100)/dpi)
            incoordy = 1*((int(instVal[k][1])+1)/dpi)
            xcoordData.put(incoordx)
            ycoordData.put(incoordy)
            uart2.write(f'{xcoordData.get()}, {ycoordData.get()}\r\n')

            hyp = sqrt(incoordx**2 + incoordy**2)
            
            thetas = np.array([1*(atan2(incoordy,incoordx)), hyp/inrad])
            
            usteps1 = int(thetas[0]*3600/(2*pi))
            usteps2 = int(thetas[1]*400/(2*pi))
            print(f'[{usteps1}, {usteps2}]')

            while True:
                stepthresh = 1
                drv1.setPos(usteps1)
                drv2.setPos(usteps2)
                sleep(.25)
                comp1 = usteps1
                comp2 = usteps2
                act1 = drv1.getPos()
                act2 = drv2.getPos()
                
                if (abs(comp1 - act1) < stepthresh) and (abs(comp2 - act2 < stepthresh)):
                    break

                else:
                    pass
            k+=1
            continue
            

    else:
        while k < len(instVal):
            incoordx = .7*((int(instVal[k][0])+100)/dpi)
            incoordy = 1*((int(instVal[k][1])+1)/dpi)
            xcoordData.put(incoordx)
            ycoordData.put(incoordy)
            uart2.write(f'{xcoordData.get()}, {ycoordData.get()}\r\n')
            hyp = sqrt(incoordx**2 + incoordy**2)

            thetas = np.array([1*(atan2(incoordy,incoordx)), hyp/inrad])
            
            usteps1 = int(thetas[0]*3600/(2*pi))
            usteps2 = int(thetas[1]*400/(2*pi))
            print(f'[{usteps1}, {usteps2}]')
            
            while True:
                stepthresh = 1
                drv1.setPos(usteps1)
                drv2.setPos(usteps2)
                sleep(.25)
                comp1 = usteps1
                comp2 = usteps2
                act1 = drv1.getPos()
                act2 = drv2.getPos()
                
                if (abs(comp1 - act1) < stepthresh) and (abs(comp2 - act2 < stepthresh)):
                    break
                    #sleep(.5)
                    pass

                else:
                    pass
            k+=1
            continue
    
    pass
if __name__ =='__main__':    
    state = 0 #INIT AND WAIT STATE
    
    #Limit switch pen up/down
    PC4 = Pin(Pin.cpu.C4, mode=Pin.OUT_PP, value=0)
    PB4 = Pin(Pin.cpu.B4, mode=Pin.OUT_PP, value=0)
    PB5 = Pin(Pin.cpu.B5, mode=Pin.OUT_PP, value=0)

    
    # Motor stuff
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
    drv1.setMinVel(0b00000000001)
    drv2.setMinVel(0b00000000001)
    
    #Set VMAX
    drv1.setMaxVel(0b00000000001)
    drv2.setMaxVel(0b00000000001)
    
    #Set Pulse div and ramp div
    drv1.setPulseramppulsediv(0b0000, 0b0000)
    drv2.setPulseramppulsediv(0b0000, 0b0000)
    
    #Set AMAX
    drv1.setMaxAcc(0b00000111,0b11010000)
    drv2.setMaxAcc(0b00000111,0b11010000)

    #Set PDIV 1 and PMUL 249
    drv1.setPMULPDIV(0b11111001, 0b0000001)
    drv2.setPMULPDIV(0b11111001, 0b0000001)
    
    #Start calib
    drv1.zeroHere()
    drv2.zeroHere()
    
    #UART Stuff
    uart2 = UART(2)
    uart2.init(115200, bits=8, parity=None, stop=1)
    
    #NewtonRaphson params
    theta_guess = np.array([5,6])
    thresh = 1e-6
    r = 0.24
    inrad = r/(pi*2)
    
    #HPGL params
    shapes = []
    
    #Name the file to be parsed and drawn, specify dpi set from Inkscape. Increaseing or decreasing
    #will change the actual size of drawing on the baord 
    file = "susamongus.hpgl"
    dpi = 240
    
    # Coop task shares and queues
    
    #used as a flag for knowing when the drawing task has finished
    drawingDone = task_share.Share ('i', thread_protect = False, name = "drawingDone")
    
    #Making use of the button on the Nucleo board to start the drawing task when pressed
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, None)
    button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, callback=onButtonpress)
    buttonFlag = task_share.Share ('i', thread_protect = False, name = "buttonFlag")
    
    #Set as 0 initially 
    buttonFlag.put(0)
    drawingDone.put(0)
    
    #Creating tasks
    drawTask = cotask.Task(drawTask, name = 'Task_2', priority = 1, 
                         period = 50, profile = True, trace = False)
    
    #Appending task to task list to be ran by scheduler based on priority
    cotask.task_list.append(drawTask)
     
    xcoordData = task_share.Queue ('f', 300, thread_protect = True, overwrite = False,
                           name = "xcoordData")
    ycoordData = task_share.Queue ('f', 300, thread_protect = True, overwrite = False,
                           name = "ycoordData")
    
    #Memory cleaning before running
    gc.collect ()
    xcoordData.clear()
    ycoordData.clear()

    #Run tasks using cotask when button is pressed. Stops the run if interrupted using ctrl+c
    while True:
        try:
            if buttonFlag.get() == 1:
                cotask.task_list.pri_sched()
            elif drawingDone.get() == 1:
                xcoordData.clear()
                ycoordData.clear()
                drv1.disable()
                drv2.disable()
                break
                
            else:
                pass
            
        except KeyboardInterrupt:
            drv1.disable()
            drv2.disable()
            PB5.low()
            break
    
    #Once the task finishes the scheduler stops being used and the motors home back to zero
        
    drv1.goZero()
    drv2.goZero()
    
    sleep(5)
    drv1.disable()
    drv2.disable()
    print('motors disabled')
