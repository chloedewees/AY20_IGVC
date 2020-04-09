#!/usr/bin/env python

""" Code written by: Dominic Larkin 15OCT2019
    
    Subscribe to pacmod can_tx and extract individual wheel speeds.    
    
"""
import rospy 
#from std_msgs.msg import Float64
#from pacmod_msgs.msg import MotorRpt1
from can_msgs.msg import Frame

#from nav_msgs.msg import Odometry
from sets import Set

import math
import tf

class WheelSpeed():

    def __init__(self):     
        rospy.init_node('wheel_speed', anonymous=True)        
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber('/pacmod/can_tx', Frame, self.update_tx)
        rospy.Subscriber('/pacmod/can_rx', Frame, self.update_rx)
        self.last_time = rospy.Time.now() 
        self.tx_set = Set()
        self.rx_set = Set()
    
    def update_tx(self, data):
        self.veh_speed = data
        self.tx_set.add(data.id)
        if (data.id == 2555):
            output_str = "Speed: " + repr(data)
            rospy.loginfo(output_str)
            
    def update_rx(self, data):
        self.veh_speed = data
        self.rx_set.add(data.id)
        if (data.id == 105):
            output_str = "Speed: " + repr(data)
            rospy.loginfo(output_str)


if __name__ == '__main__':
    try:
        veh_speed = WheelSpeed()
        rate = rospy.Rate(30) # 10hz
        tx_len = 0;
        rx_len = 0;
        while not rospy.is_shutdown():
            #v_odom.calculate_odom()
            #pub.publish(hello_str)
            if len(veh_speed.tx_set) != tx_len:
                tx_len = len(veh_speed.tx_set)
                output_str = "TX " + repr(veh_speed.tx_set) 
                rospy.loginfo(output_str)
            if len(veh_speed.rx_set) != rx_len:
                rx_len = len(veh_speed.rx_set)
                output_str = "RX " + repr(veh_speed.rx_set) 
                rospy.loginfo(output_str)
            rate.sleep()        
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Command Velocity to Odometry finished.")


'''


/pacmod/parsed_tx/steer_rpt_detail_1

header: 
  seq: 4215
  stamp: 
    secs: 1561130944
    nsecs: 981564176
  frame_id: "pacmod"
current: -0.231
position: 1.359

'''
