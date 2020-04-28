#!/usr/bin/env python

# 31 degrees traverse for each wheel

import math
import rospy
from std_msgs.msg import Int8, Float32
from pacmod_msgs.msg import PositionWithSpeed

steer_cmd = PositionWithSpeed()

#Constants
in2mm=25.4
mm2in= 1.0/in2mm
in2m= in2mm/1000.0
lbf2N=4.448
N2lbf= 1.0/lbf2N
kg2lbs=2.205
lbs2kg=1.0/kg2lbs
rpm2rad= 2.0*math.pi/60.0 #rpm to rad/s
m2miles= 2.23694 #m/s to mph
miles2m= 1.0/m2miles  #mph to m/s
phi = 1.618 #Golden Ratio
g = 9.806 #Acc due to gravity in m/s

prevTime = 0

state = 0
eDesired = 0.0
prev_e = 0.0
eDot = 0.0
eDotDesired = 0.0
kP1 = 1.5
kD1 = 0.3
v=5*miles2m
omega = 0

def state_callback(msg):
    global state
    state = msg.data

def rad2Pac(rad):
    pac_cmd = rad * 20.36
    if pac_cmd < -10.994:
        pac_cmd = -10.994
    elif pac_cmd > 10.994:
        pac_cmd = 10.994
    return pac_cmd

def steeringAngleCalculator(x):
    global prev_e, eDot, omega
    e = x.data
    currentTime = rospy.get_time() 
    dt = currentTime - prevTime
    eDot = (e - prev_e)/dt
    omega= (eDesired-e)*kP1 + (eDotDesired - eDot)*kD1 # 31deg or 0.54rad is max steering angle
    prev_e = e
    prevTime = currentTime
	

def SteeringController():
    global omega, steer_cmd
    rospy.init_node('SteeringController', anonymous=True)
    steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=10)
    rospy.Subscriber('/selfdrive/state', Int8, state_callback)
    rospy.Subscriber("/y_distance", Float32, steeringAngleCalculator)

    steer_cmd.angular_velocity_limit = 3.3  # Adjust this!!

    rate = rospy.Rate(12) #12hz
    while not rospy.is_shutdown():
        if state:
            pac_cmd = rad2Pac(omega)
            steer_cmd.angular_position = rad2Pac(omega)
            steer_pub.publish(steer_cmd)
        rate.sleep()
		
if __name__ == '__main__':
    try:
        SteeringController()
    except rospy.ROSInterruptException:
        pass
