#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int8, Float64
from pacmod_msgs.msg import PacmodCmd
import sys

def speed_callback(msg):
    global speed
    speed = msg.data
    
### Controller to halt the vehicle ###    
def stop_vehicle():
    throttle = PacmodCmd()
    throttle.f64_cmd = 0.0
    brake = PacmodCmd()
    global speed
    kp = 0.25
    rate = rospy.Rate(10)
    while speed > 0.01:
        throttle_pub.publish(throttle)
        brake_val = kp*speed
        #if brake_val > 0.8:
        #    brake_val = 0.8
        #if brake_val < 0.60:
        #    brake_val = 0.60
        brake_val = 0.75
        brake.f64_cmd = brake_val
        brake_pub.publish(brake)
        rate.sleep()
    brake.f64_cmd = 0.65
    brake_pub.publish(brake)
    return
    
def kill():
    print "\n\r*** E-Stop triggered...shutting down! ***"
    stop_vehicle()
    enable_pub.publish(False)
    ## shutdown node
    rospy.signal_shutdown("E-Stop pressed: killing ROS")
    raise KeyboardInterrupt
    sys.exit()
    
def mode_callback(msg):
    ## if kill signal received:
    if msg.data == 7:
        kill()


if __name__ == '__main__':
    speed = 0

    rospy.init_node('self_kill_node')
    throttle_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)     # using Pacmod commands
    brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
    enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
    rospy.Subscriber('/pacmod/as_tx/vehicle_speed', Float64, speed_callback)
    
    rospy.Subscriber('/gem/operation_mode', Int8, mode_callback)
    
    rospy.spin()
