from math import pi
from numpy import sign
import time
dx=0
def fixYaw(angle,angleIMU,vel):
    global dx
   
    
    val_to_add=0
    modification=0
    if (sign(fixYaw.old_angle)!=sign(angleIMU)):#if there is zero crossing
        if (angleIMU<0):
            if(vel>0):
                time.sleep(2)
                fixYaw.stathera=-2
                fixYaw.angleConstant=-1
        else: # angleIMU>0
            if(vel<0):
                time.sleep(6)
                fixYaw.stathera=0 
                fixYaw.angleConstant=1
                
    dx=angleIMU-fixYaw.old_angle
    fixYaw.old_angle=angleIMU
    #print("dx:",dx)
    return fixYaw.stathera + fixYaw.angleConstant*angle

    

        
fixYaw.yaw=0
fixYaw.old_angle=1
fixYaw.stathera=0 #sintelestis statheras
fixYaw.angleConstant=1 #sintelestis gonias



def yawSignChange(angle):
    if (sign(yawSignChange.old_angle)!=sign(angle)):
        yawSignChange.sign=yawSignChange.sign * (-1)        

    yawSignChange.old_angle=angle
    return yawSignChange.sign

yawSignChange.old=0
yawSignChange.sign=1
