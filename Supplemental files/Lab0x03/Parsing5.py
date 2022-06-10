import numpy as np
import pandas as pd
from math import cos, sin, pi, tan
import imageio
import matplotlib.pyplot as plt
from os import remove
#import TMC4210

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

# def ellipse(xc, rx, yc, ry):
#     t = np.linspace(0, 2*pi, 100)
#     bufx = []*len(t)
#     bufy = []*len(t)
    
#     for i in range(len(t)-1):
#         xval = xc + rx*cos(t[i])
#         yval = yc + ry*sin(t[i])
#         bufx.append(xval)
#         bufy.append(yval)
        
#     eReturn = np.array([bufx, bufy])
#     ellipseOut = eReturn.transpose()    
#     return ellipseOut
    
def extract(ind):
    bufx = []
    bufy = []
    
    val = ind.split(',')
    for n in range(len(val)>>1):    
        bufx.append(val[2*n])
        bufy.append(val[2*n + 1])
    instOut = np.array([bufx, bufy]).transpose()
    #print(drawsOut)
    return instOut

def draw(instVal):
    inst = instVal[0:2]
    if inst == 'PU':
        instVal.strip('PU')
        #Construct as a share later 0 = up 1 = down 
        penShare = 0
        incoordx = (int(instVal[0])+500)/dpi
        incoordy = (int(instVal[1])+500)/dpi
        thetas = iterate(NewtonRaphson, np.array([incoordx, incoordy]), theta_guess, r, thresh)
    
    elif inst == 'PD':
        instVal.strip('PD')
        #Construct as a share later 0 = up 1 = down 
        penShare = 1
        incoordx = (int(instVal[0])+500)/dpi
        incoordy = (int(instVal[1])+500)/dpi
        thetas = iterate(NewtonRaphson, np.array([incoordx, incoordy]), theta_guess, r, thresh)
    
    else:
        incoordx = (int(instVal[0])+500)/dpi
        incoordy = (int(instVal[1])+500)/dpi
        thetas = iterate(NewtonRaphson, np.array([incoordx, incoordy]), theta_guess, r, thresh)
    
    pass
if __name__ =='__main__':    
    
    theta_guess = np.array([2,8])
    thresh = 1e-6
    r = 0.24
    
    thetas1 = []
    
    shapes = []
    file = "square.hpgl"
    
    dpi = 1016
    
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
    
    shapesInst = np.vstack(shapes)
    print(shapesInst)
    
    
    sFL = len(shapesInst)
    shapesX = []*sFL
    shapesY = []*sFL
    j = 0  
    
    while j < sFL:
        shapesX.append((shapesInst[j][0]+500)/dpi)
        shapesY.append((shapesInst[j][1]+500)/dpi)
        j+=1
    sFA = np.array([shapesX, shapesY])
    shapesFlatArray = sFA.transpose()
        
    for k in range(sFL):
        thetas = iterate(NewtonRaphson, shapesFlatArray[k], theta_guess, r, thresh)
        thetasList = thetas.tolist()
        thetas1.append(thetasList)
        
    xapprox = []
    yapprox = []
    
    for m in range(len(thetas1)):
        xapprox.append(r*thetas1[m][1]*cos(thetas1[m][0]))
        yapprox.append(r*thetas1[m][1]*sin(thetas1[m][0]))
        
    approx = np.array([xapprox, yapprox])
    approx = approx.transpose()
    approx = np.array(approx)
    
    
    #this should be PC side
    filenames = []
    fig = plt.figure()
    ax = fig.add_subplot()
    
    for x in (range(sFL)):
        plt.plot([approx[x-1][0], approx[x][0]], [approx[x-1][1], approx[x][1]],color='green', marker='o', linestyle='-',
      linewidth=.5, markersize=.1)
        plt.ylim(0,6)
        plt.xlim(0,6)
        plt.title('Newton Rhapson Approximated Points From Desired Points')
        plt.xlabel('Distance X [in]')
        plt.ylabel('Distance Y [in]')
        ax.set_aspect('equal', adjustable='box')
        
        filename1 = f'{x}.png'
        filenames.append(filename1)
        plt.savefig(filename1)
        
    #Make GIF from a bunch of png files made earlier   
    with imageio.get_writer('DrawPlot.gif', mode='I') as writer:
        for filename1 in filenames:
            image = imageio.imread(filename1)
            writer.append_data(image)
            
    # Remove files
    for filename in set(filenames):
        remove(filename)
    
   