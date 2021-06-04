import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import BarycentricInterpolator

def horner(poly, n, x):
  
    # Initialize result
    result = poly[0]
  
    # Evaluate value of polynomial
    # using Horner's method
    for i in range(1, n):
  
        result = result*x + poly[i]
  
    return result


x = np.array([0, 2, 8, 9, 4, 11, 12])
y = np.array([1, 2, 3, 4, 5, 6, 7])

plt.plot(x,y)
plt.show()

# 300 represents number of points to make between T.min and T.max
xnew = np.linspace(x.min(), x.max(), 300)  
pol=np.polyfit(x, y, 5)

w=pol
xs=np.linspace(0,12,300)
ys=[]
for x in xs:
    print(x)
    y=np.polyval(w,x)
    ys.append(y)
    
zs=np.ones(len(xs))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(xs, ys, zs, 'gray')
ax.scatter3D(xs, ys, zs,cmap='Greens',s=5)
fig.show()

