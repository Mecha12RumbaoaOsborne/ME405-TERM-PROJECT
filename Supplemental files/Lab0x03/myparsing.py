import numpy as np
import pandas as pd
from math import cos, sin, pi, tan
import imageio
import matplotlib.pyplot as plt

def g(x, theta, r):
    # xInt = np.array(x)
    # xInt = xArry.astype(int)
    g1 = x[0]-r*theta[1]*float(cos(theta[0])) 
    g2 = x[1]-r*theta[1]*float(sin(theta[0]))
    return np.array([g1, g2])

def dg_dtheta(theta, r):
    return np.array([[r*theta[1]*float(sin(theta[0])), -r*float(cos(theta[0]))],
                            [-r*theta[1]*float(cos(theta[0])), -r*float(sin(theta[0]))]]) 

def NewtonRaphson (fcn, jacobian, guess, thresh):
    new_theta = np.subtract(guess, np.dot(np.linalg.inv(jacobian), fcn))
    return new_theta

def iterate(fcn, x, theta, r, thresh):
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

def ellipse(xc, rx, yc, ry):
    t = np.linspace(0, 2*pi, 100)
    bufx = []*len(t)
    bufy = []*len(t)
    
    for i in range(len(t)-1):
        xval = xc + rx*cos(t[i])
        yval = yc + ry*sin(t[i])
        bufx.append(xval)
        bufy.append(yval)
        
    eReturn = np.array([bufx, bufy])
    ellipseOut = eReturn.transpose()    
    return ellipseOut

def draw(penStat, ind):
    bufx = []
    bufy = []
    
    val = ind.strip('PUD').split(',')
    
    for n in range(len(val)>>1):
        bufx.append(int(val[2*n]))#/dpi)
        bufy.append(int(val[2*n + 1]))#/dpi)
    
    drawsOut = np.array([bufx, bufy])
    # print(drawsOut)
    return drawsOut

if __name__ =='__main__':    
    
    theta_guess = np.array([2,8])
    thresh = 1e-6
    r = 0.24
    
    shapes = []
    file = "drawing.hpgl"
    dpi = 1016
    
with open(file, 'r') as f:
    for item in f:
        command = item.split(';')

    for i in command:
        f2 = i[0:2]
        if f2 == 'PU':
            #Do pen up command
            print("pen up")
            
            if len(i) > 2:
                penDown = False
                shapes.append(draw(penDown, i))
                
        elif f2 == 'PD':
            #Do pen down command
            print("pen d")
            
            if len(i) > 2:
                penDown = True
                shapes.append(draw(penDown, i))

        else:
            # print('fook')
            None    
    
    thetas1 = []
    shapesFlat = []
    
    for sublist in shapes:
        for t1 in sublist:
            for t2 in t1:
                # shapesFlat = np.array(t2)
                shapesFlat.append(t2)
    sFL = len(shapesFlat)
    shapesX = []*486
    shapesY = []*486
    print(shapesFlat) 
