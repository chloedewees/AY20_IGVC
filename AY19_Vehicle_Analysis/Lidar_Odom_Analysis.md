# Laser Based Odom analysis using the bagfile  
gem_e2_2019-06-21-11-26-44.bag
- Note: During this run the vehicle was driven manually by use of steering wheel, gas pedal and brake pedal. We did not expect commands to be sent to the can bus we only expected to recieve data from it.

## Experiment setup in three terminals:
- Terminal 1: Run the command 'roslaunch gem_localization gem_localizer.launch'
- Terminal 2: Run the command 'roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch'  
- Terminal 3: Run the command 'rviz'
SimpleScreenRecorder was installed and recorded the three terminals and rviz.

## Params used:
Throughout the experiment many parameters were changed, mostly pointcloud_to_laserscan.  
The gem_localization repository was commited and pushed with a hash of 7b96e5ccfe6d13263584e3fa4e461647c9271b25.  

Below is the rf2o_laser_odometry.launch file:  
```html
<launch>

  <node pkg="rf2o_laser_odometry" type="rf2o_laser_odometry_node" name="rf2o_laser_odometry" output="screen">
    <param name="laser_scan_topic" value="/scan"/>        # topic where the lidar scans are being published
    <param name="odom_topic" value="/odom_rf2o" />              # topic where tu publish the odometry estimations
    <param name="publish_tf" value="true" />                   # wheter or not to publish the tf::transform (base->odom)
    <param name="base_frame_id" value="/veh_near_field"/>            # frame_id (tf) of the mobile robot base. A tf transform from the laser_frame to the base_frame is mandatory
    <param name="odom_frame_id" value="/odom" />                # frame_id (tf) to publish the odometry estimations    
    <param name="init_pose_from_topic" value="" /> # (Odom topic) Leave empty to start at point (0,0)
    <param name="freq" value="6.0"/>                            # Execution frequency.
    <param name="verbose" value="true" />                       # verbose
  </node>
  
</launch>
```

## Findings:
It appears that the lidar is not finding enough features that are stable. For example the laser returns against a hill change significantly as the car moves. Also as the car speeds up and brakes the angle of the Velodyne relevant to the ground changes slightly, thus changing the scan results.   

It appears that LiDAR based odom maybe difficult to implement in this environment.  

You can watch the final run in the video: LidarOdomAnalsis-2019-10-20_16.40.24.mkv  
In this video the red boxes are the laser scan points and are what is used to compute odometry.

This video shows the repeated message with rarely updated odom:   
ERROR: Eigensolver couldn't find a solution. Pose is not updated  




