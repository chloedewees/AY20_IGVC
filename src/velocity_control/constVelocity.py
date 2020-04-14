#!/usr/bin/env python

import rospy, sys
from std_msgs.msg import Int8, String, Float64, Bool
from pacmod_msgs.msg import PacmodCmd
state = 0
#Target Values
currentVelocity = 0.0
previousVelocity = 0.05
targetVelocity = 2.2352 #5MPH in m/s
signVelocity = 2.2352
whiteLineVelocity = 2.2352
error = 0.0
integralError = 0.0
derivativeError = 0.0
previousError = 0.0
previousIntegralError = 0.0
Kp = 0.8
Ki = 0.2
Kd = 0.0
throttle = 0.0
prevTime = 0.0
time = 0.0
controlMax = 10.0
controlMin = 0.0
MPH2MPS = 0.44704
integralMax = 2*MPH2MPS

def state_callback(msg):
    global state
    state = msg.data
    if state == 7:
        rospy.signal_shutdown('killed by selfdrive manager')
        sys.exit()
        
def speedCallBack(msg):
    global currentVelocity
    currentVelocity = msg.data
    
def whiteLineCallback(msg):
    global whiteLineVelocity
    whiteLineVelocity = msg.data
    
def signCallback(msg):
    global signVelocity
    signVelocity = msg.data


def velController():
    global currentVelocity, previousVelocity, targetVelocity, whiteLineVelocity, signVelocity, error, integralError, derivativeError, previousError, previousIntegralError, Kp, Ki, Kd, throttle, prevTime, time, controlMax, controlMin, MPH2MPS, integralMax
    prevTime = time
    time = rospy.get_time()
    dt = time-prevTime
    targetVelocity = (whiteLineVelocity + signVelocity)/2
    error = targetVelocity - currentVelocity
    previousIntegralError = integralError
    integralError = (error * (dt) + integralError)
    if integralError > integralMax:
        integralError = integralMax
    elif integralError < -integralMax:
        intregralError = -integralMax
    derivativeError = currentVelocity - previousVelocity
    ep = (Kp * error)
    ei = (Ki * integralError)
    ed = (Kd * derivativeError)
    u = ep + ei + ed
    throttle = scale(u)
    
    if u > controlMax:
        throttle = scale(controlMax)
        u -= previousIntegralError
        
    elif u < controlMin:
        throttle = scale(controlMin)
        u -= previousIntegralError
    previousError = error
    previousVelocity = currentVelocity
    return throttle
    
def scale(x):
    global controlMax
    u = (x*(1.0/controlMax))+.2
    if u > 1.0:
        u = 1.0
    return u

def constVelocity():
    global currentVelocity, previousVelocity, targetVelocity, error, integralError, derivativeError, previousError, previousIntegralError, Kp, Ki, Kd, throttle, prevTime, time, controlMax, controlMin, MPH2MPS, integralMax
    rospy.init_node('speed_controller', anonymous=True)
    accel_pub = rospy.Publisher('pacmod/as_rx/accel_cmd', PacmodCmd, queue_size = 10)
    brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size = 10)
    rospy.Subscriber('/pacmod/as_tx/vehicle_speed', Float64, speedCallBack)
    rospy.Sublisher('/speed_applied', Float64, whiteLineCallBack)
    rospy.Sublisher('/stop_speed', Float64, whiteLineCallBack)
    rospy.Subscriber('/selfdrive/state', Int8, state_callback)
    accel = PacmodCmd()
    accel.f64_cmd = 0
    brake = PacmodCmd()
    brake.f64_cmd = 0
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    
        if state and not (10<=state<20 or state==3):
            brake.f64_cmd = 0
            brake_pub.publish(brake)
            accel.f64_cmd = velController()
            accel_pub.publish(accel)
        else:
            integralError = 0
        
        rate.sleep()
        
        
if __name__ == '__main__':
    try:
        constVelocity()
    except rospy.ROSInterruptException:
        pass    
