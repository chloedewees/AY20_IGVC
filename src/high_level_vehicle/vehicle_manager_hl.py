#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Int8, Bool
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed

turn_cmd = PacmodCmd()
shift_cmd = PacmodCmd()
steer_cmd = PositionWithSpeed()

### Base settings for switching modes ###
def config(opmode=0):
    global enabled
    enable_pub.publish(True)
    ## turn off turn blinkers ##
    turn_cmd.ui16_cmd = 1
    turn_sig_pub.publish(turn_cmd)
    shift_cmd.ui16_cmd = 2
    shift_pub.publish(shift_cmd)
    steer_cmd.angular_position = 0.0
    steer_pub.publish(steer_cmd)
    ## disable Pacmod ##
    if opmode:
        tm = rospy.get_time()
        while not enabled:
            if rospy.get_time()-tm>1.0:
                print "\r\t** error: Pacmod refusing to enable..."
                raise Exception
    else:
        enable_pub.publish(False)
    return
    
def set_mode(val=0):
    global mode, speed, selfdrive_sig_tm
    if not val == mode:
        if val == 2 and speed:
            print "\n\r** vehicle still moving...cannot be put in Autonomous Self-drive mode!"
            return
        try:
            config(val)
        except:
            revert_to_manual()
            return
        mode = val
        if mode == 1:
            print "\n\r---- Tele-op ----"
            print "\r\t-- Press 'A' for forward drive and 'B' for reverse. 'RT' for throttle and 'LT' for brake. Steer with left joystick. E-Stop is 'LB'."
        elif mode == 2:
            print "\n\r---- Autonomous Self-drive ----"
        else:
            print "\n\r---- Manual Drive ----"
        config(val)
        mode_pub.publish(mode)
        selfdrive_sig_tm = rospy.get_time()
    return
    
def revert_to_manual(err_str=''):
    global mode
    if not mode == 7:
        if err_str:
            print "\r\t** {}...".format(err_str)
        set_mode(0)
    return
    
def enable_callback(msg):
    global enabled
    enabled = msg.data   

def estop_callback(msg):
    global estop, last_estop_tm
    estop = msg.data
    last_estop_tm = rospy.get_time()
    
def speed_callback(msg):
    global speed
    speed = msg.data
    
def user_input_callback(msg):
    global mode
    if not mode == 7:
        set_mode(msg.data)
                
def selfdrive_sig_callback(msg):
    global selfdrive_status, selfdrive_sig_tm
    selfdrive_status = msg.data
    if selfdrive_status == 7:
        revert_to_manual('self-drive system has died')
    elif selfdrive_status == 8:
        revert_to_manual()
    selfdrive_sig_tm = rospy.get_time()


if __name__ == '__main__':
    rospy.init_node('vehicle_manager_hl', anonymous=True)
    
    enabled, estop, speed, mode, selfdrive_status, selfdrive_sig_tm = False, False, 0.0, 0, 0, 0
    last_estop_tm = rospy.get_time()   
    
    mode_pub = rospy.Publisher('/gem/operation_mode', Int8, queue_size=10)
    enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=10)
    shift_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=10)
    steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=10)
    turn_sig_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=10)
    config()
    rospy.Subscriber('/pacmod/as_tx/enable', Bool, enable_callback)
    rospy.Subscriber('/gem/eStop', Bool, estop_callback)
    rospy.Subscriber('pacmod/as_tx/vehicle_speed', Float64, speed_callback)
    rospy.Subscriber('/gem/mode_input', Int8, user_input_callback)
    rospy.Subscriber('/selfdrive/status', Int8, selfdrive_sig_callback)
    print "\n\rStarting in Manual Drive"
    print "\n\rPress 'a' or Xbox Up for autonomous mode, 't' or Xbox Right for tele-op, or 'm' or Xbox Down for manual drive.\n\rPress 'esc' or Xbox LB to shutdown."
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    
        if estop or rospy.get_time()-last_estop_tm>0.5:
            mode = 7    # send stop and shutdown signal if estop pressed or break in estop signal
            
        elif mode and not enabled:
            revert_to_manual('possible override detected')
            
        elif mode == 2:
            if not (selfdrive_status==1 or selfdrive_status==2) and rospy.get_time()-selfdrive_sig_tm>0.2:
                revert_to_manual('self-drive status signal has died')
                
        mode_pub.publish(mode)                   
        
        rate.sleep()
