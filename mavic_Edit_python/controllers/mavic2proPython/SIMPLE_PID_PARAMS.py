QUADCOPTER_TIME_STEP=int(8)
ROVER_TIME_STEP=float(64)

TAKEOFF_THRESHOLD_VELOCITY=float(160)

floor_width=float(70)
k_vertical_thrust=float(68.5)
floor_height=float(42.4)

k_roll_p=float(10) 
waypoint_reached_tolerance=float(0.1)
k_pitch_p=float(-10)

target_altitude=float(1)

pitch_Kp=float(2)
pitch_Ki=float(0.1)
pitch_Kd=float(2)

roll_Kp=float(3)
roll_Ki=float(0.2)
roll_Kd=float(2)

throttle_Kp=float(10)
throttle_Ki=float(0.1)
throttle_Kd=float(5)

yaw_Kp=float(2)
# yaw_Ki=float(0.01)
yaw_Ki=float(0)
yaw_Kd=float(5)

yaw_setpoint=float(-0.99)
altitude_attainment_factor=float(0.8)