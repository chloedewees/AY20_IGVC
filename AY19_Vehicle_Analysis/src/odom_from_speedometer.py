#!/usr/bin/env python

""" Code written by: Dominic Larkin 15OCT2019
    
    Subscribe to sppedometer in m/s and compute odometry.    
    
"""
import rospy 
from std_msgs.msg import Float64
from pacmod_msgs.msg import MotorRpt1
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from pacmod_msgs.msg import VehicleSpeedRpt
import math
import tf

class VirtualOdom():

    def __init__(self):     
        rospy.init_node('virtual_odom', anonymous=True)        
        self.br = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("speed_odom", Odometry, queue_size=1 ) 
        #rospy.Subscriber('/pacmod/as_tx/vehicle_speed', Float64, self.update_veh_speed)
        rospy.Subscriber('/localization/imu/raw', Imu, self.update_yaw)
        rospy.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt, self.update_veh_speed_rpt)
        self.last_time = rospy.Time.now() 
        self.veh_speed = 0.0
        self.x = 0.0
        self.y = 0.0
        self.imu_data = Imu()
        self.offset_yaw = 9999.9999
    
    def update_veh_speed(self, data):
        #self.veh_speed = data.data
        pass
        
    def update_veh_speed_rpt(self, data):
        self.veh_speed = data.vehicle_speed# * 0.44704 
        
    def update_yaw(self, data):
        self.imu_data = data
        orientation = self.imu_data.orientation         
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        if (self.offset_yaw == 9999.9999): 
            self.offset_yaw = yaw
            rospy.loginfo("Yaw offset is %f.", self.offset_yaw)            
        yaw = yaw - self.offset_yaw  
        odom_msg = Odometry()
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        x_dot = self.veh_speed * math.cos(yaw)
        y_dot = self.veh_speed * math.sin(yaw)
        self.x += x_dot * dt
        self.y += y_dot * dt
        
       # Build the odometry message to publish
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = self.imu_data.orientation
        odom_msg.pose.covariance[0] = 0.2  # x
        odom_msg.pose.covariance[7] = 0.2  # y
        odom_msg.pose.covariance[35] = 0.4 # yaw
        
        odom_msg.twist.twist.linear.x = self.veh_speed
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0#angular_velocity
        #hello_str = "Speed: %f, steer: %f" % ( v_odom.veh_speed, v_odom.veh_steer)
        #rospy.loginfo(hello_str)
        self.odom_pub.publish(odom_msg)        
                
    def calculate_odom(self):
        pass
        
if __name__ == '__main__':
    try:
        v_odom = VirtualOdom()
        rate = rospy.Rate(30) # 10hz
        while not rospy.is_shutdown():
            v_odom.calculate_odom()
            #pub.publish(hello_str)
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
