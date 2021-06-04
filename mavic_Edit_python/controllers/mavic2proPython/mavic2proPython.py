from numpy import sign
from controller import *
import mavic2proHelper
from simple_pid import PID
import csv
import struct
from  math import sin,cos,pi
from SIMPLE_PID_PARAMS import *

from helping_functions import frames
from helping_functions import yawFix

k_roll_p =10
k_pitch_p=10



TIME_STEP = QUADCOPTER_TIME_STEP
TAKEOFF_THRESHOLD_VELOCITY = TAKEOFF_THRESHOLD_VELOCITY
M_PI = 3.1415926535897932384626433

robot = Robot()

[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = mavic2proHelper.getMotorAll(robot)

timestep = int(robot.getBasicTimeStep())
mavic2proMotors = mavic2proHelper.getMotorAll(robot)
mavic2proHelper.initialiseMotors(robot, 0)
mavic2proHelper.motorsSpeed(robot, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY)

front_left_led = LED("front left led")
front_right_led = LED("front right led")
gps = GPS("gps")
gps.enable(TIME_STEP)
imu = InertialUnit("inertial unit")
imu.enable(TIME_STEP)
compass = Compass("compass")
compass.enable(TIME_STEP)
gyro = Gyro("gyro")
gyro.enable(TIME_STEP)

yaw_setpoint=-1
# yaw_setpoint=1
pitchPID = PID(pitch_Kp, pitch_Ki, pitch_Kd, setpoint=0.0)
rollPID = PID(roll_Kp,roll_Ki,roll_Kd, setpoint=0.0)
throttlePID = PID(throttle_Kp,throttle_Ki,throttle_Kd, setpoint=1)
yawPID = PID(yaw_Kp,yaw_Ki,yaw_Kd, setpoint=float(yaw_setpoint))

targetX, targetY, target_altitude = 0.0	, 0.0, 1.0
targetXWorld, targetYWorld = 1.0, 0.5
print("testaki")
while (robot.step(timestep) != -1):

	led_state = int(robot.getTime()) % 2
	front_left_led.set(led_state)
	front_right_led.set(int(not(led_state)))

	roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
	pitch = imu.getRollPitchYaw()[1]
	yaw = compass.getValues()[0]
	roll_acceleration = gyro.getValues()[0]
	pitch_acceleration = gyro.getValues()[1]

	yaw_speed= gyro.getValues()[2]
	
 
	xGPS = gps.getValues()[2]
	yGPS = gps.getValues()[0]
	zGPS = gps.getValues()[1]

	#hardcoded
	# yaw=-1
	yawIMU = imu.getRollPitchYaw()[2]
	# print("yawIMU",yawIMU)
	# yaw=frames.IMUangle2world(float(yawIMU))/(2*pi)
	newCoords=frames.world2body( (targetXWorld,targetYWorld),(xGPS,yGPS) ,frames.IMUangle2world(float(yawIMU)) )
	print( "xGps:",xGPS," yGps:",yGPS," zGps:",zGPS )
	# print("IMU:",float(yawIMU)/pi,"-->",frames.IMUangle2world(float(yawIMU))/pi )
	# print("newCoords[0]:",newCoords[0,0]," newCoords[1]:",newCoords[1,0])

	yaw_fixed=yawFix.fixYaw(yaw,yawIMU,yaw_speed)
	
	throttlePID.setpoint=target_altitude
	
	vertical_input = throttlePID(zGPS)
	yaw_input = yawPID(yaw_fixed)
	
	print("angular vel:",yaw_speed)
	print("yaw fixed:",yaw_fixed)
	print("yaw:",compass.getValues()[0],"yaw_IMU:",float(yawIMU)/pi)
	# print("yaw_error:",yawPID.setpoint-yaw)
	print("yaw_input:",yaw_input)
	print("======================================================================")
	#marios
	t=robot.getTime()
	f=pow(10,-0.1)
	# targetX=sin(f*t)
	# targetY=cos(f*t)
	targetX=newCoords[1,0]
	targetY=newCoords[0,0]

	rollPID.setpoint  = targetX
	pitchPID.setpoint = targetY
 
	# roll_input  = k_roll_p  * roll + roll_acceleration + rollPID(xGPS)
	# pitch_input = k_pitch_p * pitch - pitch_acceleration + pitchPID(-yGPS)
	roll_input  = k_roll_p  * roll + roll_acceleration + rollPID(0)
	pitch_input = k_pitch_p * pitch - pitch_acceleration + pitchPID(0)

	front_left_motor_input  = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
	front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
	rear_left_motor_input   = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
	rear_right_motor_input  = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input

	mavic2proHelper.motorsSpeed(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)