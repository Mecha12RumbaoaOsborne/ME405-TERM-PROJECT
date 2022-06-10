import numpy as np
import pandas as pd
from math import cos, sin, pi, tan
import imageio
import matplotlib.pyplot as plt
from os import remove


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
    
    drawsOut = np.array([bufx, bufy]).transpose()
    #print(drawsOut)
    return drawsOut

        
 
if __name__ =='__main__':    
    
    theta_guess = np.array([2,8])
    thresh = 1e-6
    r = 0.24
    
    thetas1 = []
    
    shapes = []
    file = "circle.hpgl"
    dpi = 100
    
    with open(file, 'r') as f:
        for item in f:
            command = item.split(';')
    
        for i in command:
            f2 = i[0:2]
            if f2 == 'PU':
                #print("pen up")
                
                if len(i) > 2:
                    penDown = False
                    shapes.append(draw(penDown, i))
                    
            elif f2 == 'PD':
                #print("pen d")
                
                if len(i) > 2:
                    penDown = True
                    shapes.append(draw(penDown, i))
                    
            elif f2 == 'IN':
                #print("init")
                pass
            elif f2 == 'SP':
                #print("pen choose")
                pass
            else:
                #print('fook')
                pass
    #shapesFlat = np.hstack(shapes).transpose()
    f.close()
    shapesFlat = np.vstack(shapes)#.transpose()
    
    sFL = len(shapesFlat)
    shapesX = []*sFL
    shapesY = []*sFL
    j = 0   
    while j < sFL:
        shapesX.append((shapesFlat[j][0]+10)/dpi)
        shapesY.append((shapesFlat[j][1]+10)/dpi)
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
    approx = np.array(approx, dtype = float)
    print(approx)
    
    #this should be PC side
    filenames = []
    fig = plt.figure()
    ax = fig.add_subplot()
    
    for x in (range(sFL)):
        plt.plot([approx[x-1][0], approx[x][0]], [approx[x-1][1], approx[x][1]],color='green', marker='o', linestyle='-',
      linewidth=.5, markersize=.1)
        # plt.plot([0, approx[x][0]], [0, approx[x][1]],color='blue',linestyle='-',
        # linewidth=1.5,marker = 'o',markersize=5)
        plt.ylim(0,10)
        plt.xlim(0,10)
        plt.title('Newton Rhapson Approximated Points From Desired Points')
        plt.xlabel('Distance X [in]')
        plt.ylabel('Distance Y [in]')
        ax.set_aspect('equal', adjustable='box')
        # plt.show()
        
        # filename1 = f'{x}.png'
        # filenames.append(filename1)
        
        # plt.savefig(filename1)
        
    # with imageio.get_writer('DrawPlot.gif', mode='I') as writer:
    #     for filename1 in filenames:
    #         image = imageio.imread(filename1)
    #         writer.append_data(image)
    # # Remove files
    # for filename in set(filenames):
    #     remove(filename)
   #memory error stuff: use appropriate daa  types; dont use lists; use arrays; store data in flash than in ram;
   #pcikle? module lets you put it in a can; stroe variables in flash without having to write and read form file in flash
   #