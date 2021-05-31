#from classes import Vector
from PARAMETERS import *  
from math import sqrt
import numpy as np
import quaternion

l = L / sqrt(2)
 
def GenerateMotorCommands( collThrustCmd:float, momentCmd:np.array):  
	tor_x = momentCmd[0] /l
	tor_y = momentCmd[1] / l
	tor_z = -momentCmd[2] / kappa
	    
	f1 = (collThrustCmd + tor_x + tor_y + tor_z) / 4   # front left
	f2 = (collThrustCmd - tor_x + tor_y - tor_z) / 4   # front right
	f4 = (collThrustCmd + tor_x - tor_y - tor_z) / 4   # rear right
	f3 = (collThrustCmd - tor_x - tor_y + tor_z) / 4   # rear left 

	cmd=[0,0,0,0]
	
	cmd[0] = f1 # front left
	cmd[1] = f2 # front right
	cmd[3] = f3 # rear right
	cmd[2] = f4 # rear left

	return cmd

def BodyRateControl(pqrCmd:np.array , pqr:np.array ):
  	momentCmd=np.array([0,0,0])
   
  	I = np.array([Ixx, Iyy, Izz])
  	error = pqrCmd - pqr
  	angularAcc = kpPQR * error
  	momentCmd = I * angularAcc

  	return momentCmd

def RollPitchControl(accelCmd:np.array, attitude:np.quaternion, collThrustCmd:float ):
	pqrCmd=np.array([0,0,0])
  
	R = attitude.getRotationMatrix()

	c = -collThrustCmd / mass
	b_x = R(0, 2)
	b_y = R(1, 2)
	b_x_c = accelCmd[0] / c
	b_y_c = accelCmd[1] / c
 	b_x_err = b_x_c - b_x
   	b_y_err = b_y_c - b_y
 	b_x_dot_c = kpBank * b_x_err
   	b_y_dot_c = kpBank * b_y_err
 	k = 1 / R(2, 2)
  	pqrCmd[0] = k * (R(1, 0) * b_x_dot_c - R(0, 0) * b_y_dot_c)
  	pqrCmd[1] = k * (R(1, 1) * b_x_dot_c - R(0, 1) * b_y_dot_c)

	return pqrCmd

def BodyRateControl(pqrCmd:np.array,pqr:np.array):

	"""
	// Calculate a desired 3-axis moment given a desired and current body rate
	// INPUTS: 
	//   pqrCmd: desired body rates [rad/s]
	//   pqr: current or estimated body rates [rad/s]
	// OUTPUT:
	//   return a V3F containing the desired moments for each of the 3 axes	
	"""
 
  	momentCmd=np.array([0,0,0])
   
	I = np.array([Ixx, Iyy, Izz])
  	error = pqrCmd - pqr
  	angularAcc = kpPQR * error
  	momentCmd = I * angularAcc
    

  	return momentCmd

def RollPitchControl(accelCmd:np.array, attitude:np.quaternion,collThrustCmd:float):
	"""
 	// Calculate a desired pitch and roll angle rates based on a desired global
	//   lateral acceleration, the current attitude of the quad, and desired
	//   collective thrust command
	// INPUTS: 
	//   accelCmd: desired acceleration in global XY coordinates [m/s2]
	//   attitude: current or estimated attitude of the vehicle
	//   collThrustCmd: desired collective thrust of the quad [N]
	// OUTPUT:
	//   return a V3F containing the desired pitch and roll rates. The Z
	//     element of the V3F should be left at its default value (0)	
	// HINTS: 
	//  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
	//  - you'll need the roll/pitch gain kpBank
	//  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first
	"""
 
	pqrCmd=np.array([0,0,0])
  	R = attitude.getRotationMatrix()

	
	c = -collThrustCmd / mass
	b_x = R(0, 2)
	b_y = R(1, 2)
	b_x_c = accelCmd[0] / c
	b_y_c = accelCmd[1] / c
	b_x_err = b_x_c - b_x
	b_y_err = b_y_c - b_y
	b_x_dot_c = kpBank * b_x_err
	b_y_dot_c = kpBank * b_y_err

  	k = 1 / R(2, 2)
  	pqrCmd[0] = k * (R(1, 0) * b_x_dot_c - R(0, 0) * b_y_dot_c)
  	pqrCmd[1] = k * (R(1, 1) * b_x_dot_c - R(0, 1) * b_y_dot_c)

 	return pqrCmd


def AltitudeControl(posZCmd:float,velZCmd:float,posZ:float, velZ:float, attitude:np.quaternion,accelZCmd:float, dt:float):
 	"""
 	// Calculate desired quad thrust based on altitude setpoint, actual altitude,
	//   vertical velocity setpoint, actual vertical velocity, and a vertical 
	//   acceleration feed-forward command
	// INPUTS: 
	//   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
	//   posZ, velZ: current vertical position and velocity in NED [m]
	//   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
	//   dt: the time step of the measurements [seconds]
	// OUTPUT:
	//   return a collective thrust command in [N]	
	"""
 
  	R = attitude.getRotationMatrix()
   
	thrust = 0
	z_err = posZCmd - posZ
	z_dot_err = velZCmd - velZ
	integratedAltitudeError += z_err * dt

 	net_z_dot_dot = kpPosZ * z_err + kpVelZ * z_dot_err + KiPosZ * integratedAltitudeError
	net_z_dot_dot = CONSTRAIN(net_z_dot_dot, -maxAscentRate / dt, maxDescentRate / dt)
	u1_bar = net_z_dot_dot - float(CONST_GRAVITY)
	acc = u1_bar / R(2, 2)
	thrust = -acc * mass

 	return thrust

def LateralPositionControl(posCmd:np.array,velCmd:np.array, pos:np.array,vel:np.array, accelCmd:np.array):
	"""
 	// Calculate a desired horizontal acceleration based on 
	//  desired lateral position/velocity/acceleration and current pose
	// INPUTS: 
	//   posCmd: desired position, in NED [m]
	//   velCmd: desired velocity, in NED [m/s]
	//   pos: current position, NED [m]
	//   vel: current velocity, NED [m/s]
	//   accelCmd: desired acceleration, NED [m/s2]
	// OUTPUT:
	//   return a V3F with desired horizontal accelerations. 
	//     the Z component should be 0
	// HINTS: 
	//  - use fmodf(foo,b) to constrain float foo to range [0,b]
	//  - use the gain parameters kpPosXY and kpVelXY
	//  - make sure you cap the horizontal velocity and acceleration
	//    to maxSpeedXY and maxAccelXY	
	// make sure we don't have any incoming z-component
	"""
 
	accelCmd[2] = 0
	velCmd[2] = 0
	posCmd[2] = pos[2]
	
	velCmd[0] = CONSTRAIN(velCmd[0], -maxSpeedXY, maxSpeedXY)
	velCmd[1] = CONSTRAIN(velCmd[1], -maxSpeedXY, maxSpeedXY)
	posError=(posCmd - pos)
	velError=(velCmd - vel)
	kpPos=np.array([ kpPosXY, kpPosXY, 0])
	kdPos=np.array([kpVelXY, kpVelXY, 0])
 
	accelCmd = kpPosXY * posError + kdPos * velError + accelCmd
	accelCmd[0] = CONSTRAIN(accelCmd[0], -maxAccelXY, maxAccelXY)
	accelCmd[1] = CONSTRAIN(accelCmd[1], -maxAccelXY, maxAccelXY)

 	return accelCmd

def YawControl(yawCmd:float ,yaw:float):
	"""
	// Calculate a desired yaw rate to control yaw to yawCmd
	// INPUTS: 
	//   yawCmd: commanded yaw [rad]
	//   yaw: current yaw [rad]
	// OUTPUT:
	//   return a desired yaw rate [rad/s]
	// HINTS: 
	//  - use fmodf(foo,b) to constrain float foo to range [0,b]
	//  - use the yaw control gain parameter kpYaw
	"""
	yawRateCmd=0
	yaw_err = yawCmd - yaw
	yawRateCmd = kpYaw * yaw_err

   	return yawRateCmd


def RunControl(dt:float ,simTime:float ,attitude:np.quaternion):
    
	curTrajPoint = GetNextTrajectoryPoint(simTime)

  	collThrustCmd = AltitudeControl(curTrajPoint.position[2], curTrajPoint.velocity[2], estPos[2], estVel[2], attitude, curTrajPoint.accel[2], dt)

  	#// reserve some thrust margin for angle control
  	thrustMargin = 0.1*(maxMotorThrust - minMotorThrust)
  	collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4, (maxMotorThrust-thrustMargin)*4)
  
  	desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel)
  
  	desOmega = RollPitchControl(desAcc, attitude, collThrustCmd)
  	desOmega[2] = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw())

  	desMoment = BodyRateControl(desOmega, estOmega)

  	return GenerateMotorCommands(collThrustCmd, desMoment)