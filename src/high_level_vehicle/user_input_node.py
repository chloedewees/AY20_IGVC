#!/usr/bin/env python

import sys, select, termios, tty, os, time
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy
    
def delay(dtm):
  ctm = rospy.get_time()
  while rospy.get_time()-ctm < dtm:
    pass
  return

def mode_callback(msg):
    global mode, shutdown
    mode = msg.data
    if mode == 7:   # if e-stop is pressed
        shutdown = True

def joy_callback(msg):
    global mode, last_joy_tm, shutdown
    tm = rospy.get_time()
    if tm-last_joy_tm>0.2:  # implement debounce
        last_joy_tm = tm
        # if LB pressed signal shutdown
        if msg.buttons[4]:
            hl_mode_input_pub.publish(7)
            shutdown = True
        # if down arrow pressed put in manual drive
        elif msg.axes[7] == -1.0:
            hl_mode_input_pub.publish(0)
        # if right arrow pressed put in tele-op
        elif msg.axes[6] == -1.0:
            hl_mode_input_pub.publish(1)
        # if up arrow pressed put in self drive
        elif msg.axes[7] == 1.0:
            hl_mode_input_pub.publish(2)
        elif mode == 2:
            if msg.buttons[0]:
                sd_user_input_pub.publish(13)
            elif msg.buttons[1]:
                sd_user_input_pub.publish(72)
            elif msg.buttons[2]:
                sd_user_input_pub.publish(67)
        
def cleanup():
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    mode, last_joy_tm, shutdown = 0, 0.0, False
    rospy.init_node("user_input_publisher")
    hl_mode_input_pub = rospy.Publisher('/gem/mode_input', Int8, queue_size=1)
    sd_user_input_pub = rospy.Publisher('/selfdrive/user_input', Int8, queue_size=1)
    rospy.Subscriber('/gem/operation_mode', Int8, mode_callback)
    rospy.Subscriber('/joy', Joy, joy_callback)
    
    ### Key press code adapted from https://www.darkcoding.net/software/non-blocking-console-io-is-not-possible/ ###
    key_delay = 0.2
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    rospy.on_shutdown(cleanup)
    tty.setcbreak(sys.stdin.fileno())
    while not rospy.is_shutdown():
        try:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                char = sys.stdin.read(1)
                # if back arrow pressed signal shutdown
                if char in '\x1b':
                    hl_mode_input_pub.publish(7)
                    shutdown = True
                # if down arrow pressed put in manual drive
                elif char in 'mM':
                    hl_mode_input_pub.publish(0)
                    delay(key_delay)
                # if right arrow pressed put in tele-op
                elif char in 'tT':
                    hl_mode_input_pub.publish(1)
                    delay(key_delay)
                # if up arrow pressed put in self drive
                elif char in 'aAsS':
                    hl_mode_input_pub.publish(2)
                    delay(key_delay)
                elif mode == 2:
                    sd_user_input_pub.publish(ord(char))
        except:
            pass
        if shutdown:
            #print 'user input node shutting down'
            rospy.signal_shutdown("shuting down user input node")
            sys.exit()
        
