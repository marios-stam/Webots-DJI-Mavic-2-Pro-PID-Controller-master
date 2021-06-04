from math import pi
from numpy import sign
dx=0
def fixYaw(angle,vel):
    global dx
    print("dx:",dx)
    val_to_add=0
    modification=0
    if (sign(fixYaw.old_angle)!=sign(angle)):#if there is zero crossing
        if(angle>0):
            if( vel>0):
                print("thetiki gonia - dx>0")
                fixYaw.PiSign=0
                fixYaw.AngleSign=1
                val_to_add=pi       
            else:
                print("thetiki gonia - dx<0")
                fixYaw.PiSign=1
                fixYaw.AngleSign=1
                val_to_add=-pi
        else:
            if( vel>0):
                print("arnitiki gonia - dx>0")
                val_to_add=pi
                fixYaw.PiSign=1
                fixYaw.AngleSign=-1        
            else:
                print("arnitiki gonia - dx<0")
                val_to_add=-pi
                fixYaw.PiSign=0
                fixYaw.AngleSign=1
            
    fixYaw.yaw=fixYaw.yaw+val_to_add
    fixYaw.old_angle=angle
    dx=angle-fixYaw.old_angle

    return fixYaw.yaw + fixYaw.PiSign*pi + fixYaw.AngleSign*angle
    
fixYaw.yaw=0
fixYaw.old_angle=0
fixYaw.dx=0
fixYaw.PiSign=0
fixYaw.AngleSign=1

def yawSignChange(angle):
    if (sign(yawSignChange.old_angle)!=sign(angle)):
        yawSignChange.sign=yawSignChange.sign * (-1)        

    yawSignChange.old_angle=angle
    return yawSignChange.sign

yawSignChange.old=0
yawSignChange.sign=1
