import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import BarycentricInterpolator
import matplotlib.pyplot as plt
import cv2

n=200
pol_degree=3

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
    print(x)
    print(y)
    print(z)
    
    poly=np.polyfit(x, y, pol_degree)
    polz=np.polyfit(x, z, pol_degree)
    
    xs=np.linspace(min(x),max(x),n)
    ys=[]
    zs=[]
    for x in xs:
        y=np.polyval(poly,x)
        z=np.polyval(polz,x)
        ys.append(y)
        zs.append(z)

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
    
# x = np.array([0, 2, 8, 9, 4, 11, 12])
# y = np.array([1, 2, 3, 4, 5, 6, 7])
x=[0,1]
y=[0,1]
z=[0,1]


(xs, ys, zs)=generateTrajectory(x,y,z,pol_degree,n)
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
