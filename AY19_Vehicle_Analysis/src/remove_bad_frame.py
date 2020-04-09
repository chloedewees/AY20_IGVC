#!/usr/bin/env python

""" Code written by: Dominic Larkin 15OCT2019
    
    Subscribe to sppedometer in m/s and compute odometry. 
    
/localization/fix
/localization/gps
/localization/imu/raw

/localization/imu_data_str
/localization/magnetic
/localization/speed_odom
/localization/velocity

    
       
    
"""
import rosbag

bag = rosbag.Bag('/home/user1/Data/gem_nav_odom_2019-06-21-11-26-44/gem_e2_2019-06-21-11-26-44.bag')
with rosbag.Bag('output.bag', 'w') as outbag:
    for topic, msg, t in bag.read_messages():
        try:     
            if msg.header.frame_id == '/gps':
                msg.header.frame_id = 'gps'
        except:
            print("%s Does not have a frame_id\n" ,topic )
        outbag.write(topic, msg, t)

bag.close()
