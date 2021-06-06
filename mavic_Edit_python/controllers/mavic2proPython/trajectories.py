import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import BarycentricInterpolator,UnivariateSpline
import matplotlib.pyplot as plt
import cv2

n=200
pol_degree=2

points=[]

def horner(poly, n, x):
  
    # Initialize result
    result = poly[0]
  
    # Evaluate value of polynomial
    # using Horner's method
    for i in range(1, n):
        result = result*x + poly[i]
  
    return result


def getXYZafterclick(event):
        global ax,fig
        global x,y
        s=ax.format_coord(event.xdata,event.ydata)
        
        s=s.replace("x", "")
        s=s.replace("y", "")
        s=s.replace("z", "")
        s=s.replace("=", "")
        s=s.replace(" ", "")
        s=s.split(",")

        
        coords=[]
        for i in s:
            sign=1
            if "−" in i:
                i=i.replace("−","")
                sign=-1
            coords.append(sign*float(i))
        
        print("egine mala")
           
            
        print(coords)
        points.append(coords)
        plt.show()
        
        x.append(coords[0])
        y.append(coords[1])
        z.append(coords[2])
        
        print("x:",x)
        print("y:",y)
        
        updateTrajectory()
        return coords
        
def generateTrajectory(x,y,z,pol_degree, n):
    """
    INPUT:
    n:number of desired points
    """
    print("points #:",len(x))
    print("x:",x)
    print("y:",y)
    print("z:",z)
 
    xs=np.linspace(min(x),max(x),n)
    ys=[]
    zs=[]

    numberOfInputPoints=len(x)
    InputpointsPerTraj=4

    subTrajectoriesNumber=int(numberOfInputPoints/InputpointsPerTraj)
    remainingPointsNumber=numberOfInputPoints%InputpointsPerTraj
    if (subTrajectoriesNumber==0):
        subTrajectoriesNumber=1
        
    PlottedpointsPerSubtrajectory=round( n/subTrajectoriesNumber )
    plottedPointsbetween2InputPoints=n/numberOfInputPoints
    
    print("subTrajectoriesNumber:",subTrajectoriesNumber)
    print("PlottedpointsPerSubtrajectory:",PlottedpointsPerSubtrajectory)
    print("plottedPointsbetween2InputPoints:",plottedPointsbetween2InputPoints)
    
    i=0
    while ((i+1)*InputpointsPerTraj<=numberOfInputPoints):
        print("============================================================")
        print("i:",i)
        print("numberOfInputPoints",numberOfInputPoints)
        
        if (numberOfInputPoints-i>=InputpointsPerTraj):#an yparxei diathesimo to epomeno block
            continuity=0 if i==0 else 1
            continuity_added=int(continuity*plottedPointsbetween2InputPoints)
            
            print("checked yi:",numberOfInputPoints-InputpointsPerTraj*(i+1)+continuity,numberOfInputPoints-InputpointsPerTraj*(i)+continuity)
            print("checked ypoints:",n-PlottedpointsPerSubtrajectory*(i+1)+continuity_added,n-PlottedpointsPerSubtrajectory*i+continuity_added)
            print("checked ypoints remaining:",n-plottedPointsbetween2InputPoints*(InputpointsPerTraj-1)*(i+1)+continuity_added,n-plottedPointsbetween2InputPoints*InputpointsPerTraj*(i)+continuity_added)

            
            print("continuity:",continuity)
            xi=x[numberOfInputPoints-InputpointsPerTraj*(i+1)+continuity:numberOfInputPoints-InputpointsPerTraj*(i)+continuity]
            yi=y[numberOfInputPoints-InputpointsPerTraj*(i+1)+continuity:numberOfInputPoints-InputpointsPerTraj*(i)+continuity]
            zi=z[numberOfInputPoints-InputpointsPerTraj*(i+1)+continuity:numberOfInputPoints-InputpointsPerTraj*(i)+continuity]

            print(xi)
            print(yi)
            print(zi)
            
            poly=np.polyfit(xi, yi, pol_degree)
            polz=np.polyfit(xi, zi, pol_degree)

            if (remainingPointsNumber==0):
                print("prosoxi re mlka")
                inputCoords=xs[n-PlottedpointsPerSubtrajectory*(i+1)+continuity_added:n-PlottedpointsPerSubtrajectory*i+continuity_added]
            else:
                inputCoords=xs[int(n-plottedPointsbetween2InputPoints*(InputpointsPerTraj-1)*(i+1)+continuity_added):int(n-plottedPointsbetween2InputPoints*(InputpointsPerTraj-1)*(i)+continuity_added)]
            #print("inputCoords:",inputCoords)

            yPoints=np.polyval(poly,inputCoords)
            zPoints=np.polyval(polz,inputCoords)

            for j in range(len(yPoints)):
                ys.append(yPoints[j])
                zs.append(zPoints[j])
        i=i+1
        print("============================================================")
        
    print("remaining:",numberOfInputPoints-i*InputpointsPerTraj,remainingPointsNumber)
    #==========remaining points==========
    if(remainingPointsNumber>0):
        print("opa")
        print("plottedPointsbetween2InputPoints:",plottedPointsbetween2InputPoints)
        print("inputCoords_remaining:",(remainingPointsNumber+1)*plottedPointsbetween2InputPoints)
        remainingPoints=xs[0:remainingPointsNumber]
        xi=x[0:remainingPointsNumber+1]
        yi=y[0:remainingPointsNumber+1]
        zi=z[0:remainingPointsNumber+1]

        poly=np.polyfit(xi, yi, 3)
        polz=np.polyfit(xi, zi, 3)
        continuity_added=int(continuity*plottedPointsbetween2InputPoints)
        inputCoords=xs[0:int( (remainingPointsNumber+1)*plottedPointsbetween2InputPoints)]

        
        yPoints=np.polyval(poly,inputCoords)
        zPoints=np.polyval(polz,inputCoords)

        for j in range(len(yPoints)):
            ys.append(yPoints[j])
            zs.append(zPoints[j])
    #/==========remaining points==========
    return (xs, ys, zs)

def updateTrajectory():
    global ax,fig
    global x,y,z
    global n,pol_degree

    (xs, ys, zs)=generateTrajectory(x,y,z,pol_degree, n)
    
    ax.plot3D(xs, ys, zs, 'red')
    for coords in points:
        ax.scatter3D(coords[0], coords[1], coords[2],cmap='Greens')

    ax.set_xticks([0,2,4,6,8,10])
    ax.set_yticks([0,2,4,6,8,10])
    ax.set_zticks([0,2,4,6,8,10])

    ax.axes.set_xlim3d(left=0.2, right=9.8) 
    ax.axes.set_ylim3d(bottom=0.2, top=9.8) 
    ax.axes.set_zlim3d(bottom=0.2, top=9.8) 

    plt.show()
    



if __name__=="main":
    # x = np.array([0, 2, 8, 9, 4, 11, 12])
    # y = np.array([1, 2, 3, 4, 5, 6, 7])
    x=[0,0.5,1,1,1.5]
    y=[0,0.5,0.7,1,0.5]
    z=[0,0.5,2,1,0.5]    
    (xs, ys, zs)=generateTrajectory(x,y,z,pol_degree,n)
    print("xs:",len(xs))
    print("ys:",len(ys)) 
    print("zs:",len(zs))
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.set_xticks([0,2,4,6,8,10])
    ax.set_yticks([0,2,4,6,8,10])
    ax.set_zticks([0,2,4,6,8,10])

    ax.axes.set_xlim3d(left=0.2, right=9.8) 
    ax.axes.set_ylim3d(bottom=0.2, top=9.8) 
    ax.axes.set_zlim3d(bottom=0.2, top=9.8) 

    ax.plot3D(xs, ys, zs, 'gray')
    #ax.scatter3D(xs, ys, zs,cmap='Greens',s=0)


    fig.canvas.callbacks.connect('button_press_event', getXYZafterclick)

    plt.show()
