# AY20_IGVC
Repo for Self-Drive Capstone, AY20

ROS Version: Kinetic 
Python 2.7.12
Ubuntu 16.04

Team Lead: Christopher Little - Christopher.little@westpoint.edu

## Getting ROS to recognize a new workspace (for example, catkin_ws):
*In terminal, run 'sudo gedit ~/.bashrc'
*At the bottom of the file, add the line 'source [YOUR PATH]/catkin_ws/devel/setup.bash'
*Save and exit, then new terminal windows

## Getting darknet to run on a ROS machine:

*In terminal, run 'sudo gedit /etc/ld.so.conf.d/darknetLib.conf'

*Then, paste the path to the libdarknet.so file, e.g., '/home/user1/catkin_ws/src/AY20_IGVC/src/object_recognition/darknet_test'

*Save and exit, then run 'sudo ldconfig' in the terminal.

*Now new terminal windows will work with it.
