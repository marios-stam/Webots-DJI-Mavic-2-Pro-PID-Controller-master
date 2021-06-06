
import numpy as np
import matplotlib.pyplot as plt

def CatmullRomSpline(P0, P1, P2, P3, a, nPoints=50):
  """
  P0, P1, P2, and P3 should be (x,y) point pairs that define the Catmull-Rom spline.
  nPoints is the number of points to include in this curve segment.
  """
  # Convert the points to numpy so that we can do array multiplication
  P0, P1, P2, P3 = map(np.array, [P0, P1, P2, P3])

  # Calculate t0 to t4
  alpha = a
  def tj(ti, Pi, Pj):
    xi, yi = Pi
    xj, yj = Pj
    return ( ( (xj-xi)**2 + (yj-yi)**2 )**0.5 )**alpha + ti

  t0 = 0
  t1 = tj(t0, P0, P1)
  t2 = tj(t1, P1, P2)
  t3 = tj(t2, P2, P3)

  # Only calculate points between P1 and P2
  t = np.linspace(t1,t2,nPoints)

  # Reshape so that we can multiply by the points P0 to P3
  # and get a point for each value of t.
  t = t.reshape(len(t),1)

  A1 = (t1-t)/(t1-t0)*P0 + (t-t0)/(t1-t0)*P1
  A2 = (t2-t)/(t2-t1)*P1 + (t-t1)/(t2-t1)*P2
  A3 = (t3-t)/(t3-t2)*P2 + (t-t2)/(t3-t2)*P3

  B1 = (t2-t)/(t2-t0)*A1 + (t-t0)/(t2-t0)*A2
  B2 = (t3-t)/(t3-t1)*A2 + (t-t1)/(t3-t1)*A3

  C  = (t2-t)/(t2-t1)*B1 + (t-t1)/(t2-t1)*B2
  return C

def CatmullRomChain(P,alpha):
  """
  Calculate Catmull Rom for a chain of points and return the combined curve.
  """
  sz = len(P)

  # The curve C will contain an array of (x,y) points.
  C = []
  for i in range(sz-3):
    c = CatmullRomSpline(P[i], P[i+1], P[i+2], P[i+3],alpha)
    C.extend(c)

  return C

def getXYZafterclick(event):
        global ax,fig
        global Points
        s=ax.format_coord(event.xdata,event.ydata)
        
        s=s.replace("x", "")
        s=s.replace("y", "")
        s=s.replace("z", "")
        s=s.replace("=", "")
        #s=s.replace(" ", "")
        s=s.split(" ")

        
        coords=[]
        for i in s:
            sign=1
            if "−" in i:
                i=i.replace("−","")
                sign=-1
            coords.append(sign*float(i))
        
        Points.pop()
        Points.append(np.array(coords))
        Points.append(np.array([coords[0]+.01,coords[1]+.01]) )
        updateTrajectory()
        return coords

def generateTraj(Points,a):
    c = CatmullRomChain(Points,a)
    x,y = zip(*c)
    return (x,y)

def updateTrajectory():
    global ax
    global Points,a
    plt.cla()
    # ax.set_xticks([0,2,4,6,8,10])
    # ax.set_yticks([0,2,4,6,8,10])
    # ax.set_xlim(10)
    # ax.set_xmargin
    # ax.set_ylim(10)
    x,y=generateTraj(Points,a)
    ax.plot(x,y,c='blue')

    # Plot the control points
    px, py = zip(*Points)
    plt.plot(px[1:-1],py[1:-1],'o',c='black')
    plt.grid(b=True)
    plt.show()

def getTrajPoints():
    x,y=generateTraj(Points,a)
    trajpoints=[]
    for i in range(len(x)):
        trajpoints.append([x[i],y[i]])

    return trajpoints

def drawActualvsDesiredTraj(actualCoords):
	xAct,yAct=zip(*actualCoords)
	fig = plt.figure()
	ax = plt.axes()
	ax.plot(xAct,yAct,c='red')	
	x,y=generateTraj(Points,a)
	ax.plot(x,y,c='green')
	plt.show()
    
# Define a set of points for curve to go through
Points = np.random.rand(10,2)
#Points=array([array([153.01,722.67]),array([152.73,699.92]),array([152.91,683.04]),array([154.6,643.45]),
#        array([158.07,603.97])])
Points = np.array([np.array([0.5,0]),
              np.array([3,0]),
              np.array([3,1])
              ])

x1=Points[0][0]
x2=Points[1][0]
y1=Points[0][1]
y2=Points[1][1]
x3=Points[-2][0]
x4=Points[-1][0]
y3=Points[-2][1]
y4=Points[-1][1]

Points=list(Points)
Points.insert(0,np.array([x1+.01,y1+.01]))
Points.append(np.array([x2+.01,y2+.01]))

print ( "# of points:",len(Points))

fig = plt.figure()
ax = plt.axes()
fig.canvas.callbacks.connect('button_press_event', getXYZafterclick)

# ax.set_xticks([0,2,4,6,8,10])
# ax.set_yticks([0,2,4,6,8,10])
# ax.set_xlim(10)
# ax.set_ylim(10)

a=0.5
x,y=generateTraj(Points,a)
plt.plot(x,y,c='blue')

# Plot the control points
px, py = zip(*Points)
plt.plot(px,py,'o',c='black')

plt.grid(b=True)
plt.show()

