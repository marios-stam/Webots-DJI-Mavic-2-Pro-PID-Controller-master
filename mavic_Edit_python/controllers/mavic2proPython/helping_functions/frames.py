from math import sin, cos, pi
import numpy as np
def body2world(xb,yb,theta):
    R=np.array([
            [cos(theta),0,0],
            [sin(theta),0,0],
            [    0     ,0,1]
                ])
    body=np.array([xb,yb,theta])
    return R*body


def world2body(worldCoords,robotCoords,theta):
    """
    INPUT:
    worldCoords ->(xw,yw):coordinates of point on world frame 
    robotCoords ->(xr,yr):coordinates of robot on world frame 
    theta:yaw angle of robot

    OUTPUT:
    bodyCoords ->(xb,yb) :coordinates of point on body frame 
    """
    (xw,yw)=worldCoords
    (xr,yr)=robotCoords
    
    R=np.array([
            [cos(theta),-sin(theta),xr],
            [sin(theta),cos(theta) ,yr],
            [    0     ,     0     ,1]
                ])
    
    R_inv=np.linalg.inv(R)
    #print("R=",R)
    #print("Rinv=",R_inv)
    world=np.array([
                    [xw],
                    [yw],
                    [1]
                       ])

    return np.matmul(R_inv, world)[:2]

def IMUangle2world(IMUangle):
    """
    INPUT:
    IMUangle   :yaw angle given by IMU
      
    OUTPUT: 
    worldAngle :yaw angle at world frame 
    """
    worldAngle=IMUangle+ pi/2
    return worldAngle
    
xw,yw=1,1
theta=3.14
print(world2body( (xw,yw),(0,0) ,theta))
