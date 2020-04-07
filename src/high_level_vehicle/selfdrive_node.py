#!/usr/bin/env python

import rospy, sys
from std_msgs.msg import Int8, UInt16MultiArray, Float64, Bool
from AY20_IGVC.msg import VisualObject, VisualObjectArray
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, SystemRptInt

shift_cmd = PacmodCmd()
brake_cmd = PacmodCmd()

run_modes = {'\n':[0,"Regular Self-drive"],'q1':[1,'E-Stop Qualification'],'q2':[2,'Straight Lane Keeping Qualification'],\
            'q3':[3,'Left Turn Qualification'],'q4':[4,'Right Turn Qualification'], 'q5':[5,'Stop Sign Detection Qualification']}

def mode_config(opt=0):
    pass #...
    return

def show_options():
    print  "\r\t\tOPTIONS:\
            \n\r\t\t * E-Stop Qualification:                'q1'\
            \n\r\t\t * Straight Lane Keeping Qualification: 'q2'\
            \n\r\t\t * Left Turn Qualification:             'q3'\
            \n\r\t\t * Right Turn Qualification:            'q4'\
            \n\r\t\t * Stop Sign Detection Qualification    'q5'"
    return

def countdown_to_run(mode_nm):
    global run_status, shift_cmd, brake_cmd, tgt_shift
    print "\r\t-- running {} in\n\r\t\t5...".format(mode_nm)
    tgt_shift = 3    # put in forward drive
    shift_cmd.ui16_cmd = tgt_shift
    shift_pub.publish(shift_cmd)
    tm = rospy.get_time()
    n = 4
    while n > -1:
        if not run_status == 2:
            return
        elif rospy.get_time()-tm>1.0:
            tm = rospy.get_time()
            if not n:
                if shift_state == 3:
                    print "\r\t-- RUNNING!"
                    brake_cmd.f64_cmd = 0
                    brake_pub.publish(brake_cmd)
                    run_status = 3
                else:
                    print "\r\t** error: not shifting to forward drive..."
                    run_status = 8
            else:
                print "\r\t\t{}...".format(n)
            n -= 1  
    return
    
def prompt_user(arg=0):
    if not arg:
        print "\t-- enter run mode (ENTER or A for default self drive, 'h' or B for other options, 'c' or X to cancel): "
    return
    
def user_input_callback(msg):
    global run_status, runmode_key, runmode, sd_state
    val = msg.data
    if val == 67 or val == 99:  # if 'c' key or X button
        run_status = 8
        print "\r\t** canceled by operator..."
    elif run_status == 1:
        if val == 72 or val == 104:   # if 'h' key
            show_options()
            prompt_user()
        elif val == 13:
            runmode_key = '\n'
        else:
            runmode_key += chr(val)
        if len(runmode_key) == 2 or runmode_key == '\n':
            try:
                runmode = run_modes[runmode_key]
                mode_config(runmode[0])
                run_status = 2
            except KeyError:
                print "\r\t\t?? '{}' is not a valid option".format(runmode_key)
                prompt_user()
            runmode_key = ''
    elif run_status == 3:
        if val == 98:
            sd_state = 11
            print "\r\t-- applying brake controller"
     
def hl_mode_callback(msg):
    global run_status
    mode = msg.data
    if msg.data == 7:
        run_status = 7
    elif not run_status and msg.data == 2:
        run_status = 1
        prompt_user(0)
    elif run_status and not msg.data == 2:
        run_status = 0
        cleanup()
        
def shift_rpt_callback(msg):
    global shift_state, shift_cmd
    shift_state = msg.output
    if run_status and not shift_state == tgt_shift:
        shift_cmd.ui16_cmd = tgt_shift
        shift_pub.publish(shift_cmd)    
        
def cnn_callback(msg):
    global visual_objects
    objects = []
    for obj in msg.objects:
        objects.append((obj.type, obj.xloc, obj.yloc, obj.area))
    obj_dict = {}
    visual_objects = objects    

### Primary Arbitrator that determines when to take certain actions by changing states ###   
def decide_state(state, objects):
    if 0 in objects:
        return 11
    else:
        return 1    
        
def cleanup():
    global shift_state, sd_state
    sd_state = 0
    shift_state = 2
    return
                    
def if_killed():
    status_pub.publish(7)   # Broadcast a shutdown
    state_pub.publish(7)
    
### Primary Arbitrator that determines when to take certain actions by changing states ###   
def decide_state(state, objects):
    #if 0 in objects:
        #return 11
    #else:
    if state == 11:
        return state
    else:
        return 1

if __name__ == '__main__':
    sd_state, run_status, runmode_key, runmode, tgt_shift, shift_state, visual_objects = 0, 0, '', [], 2, 2, []
    
    rospy.init_node('self_drive_manager', anonymous=True)
    rospy.on_shutdown(if_killed)
    state_pub = rospy.Publisher('/selfdrive/speed_state', Int8, queue_size=10)
    pacmod_steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=10)
    shift_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=10)
    status_pub = rospy.Publisher('/selfdrive/status', Int8, queue_size=10)
    state_pub = rospy.Publisher('/selfdrive/state', Int8, queue_size=10)
    brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=10)
    rospy.Subscriber('/gem/operation_mode', Int8, hl_mode_callback)
    rospy.Subscriber('/selfdrive/user_input', Int8, user_input_callback)
    rospy.Subscriber('/pacmod/parsed_tx/shift_rpt', SystemRptInt, shift_rpt_callback)
    rospy.Subscriber('/visual_objects', VisualObjectArray, cnn_callback)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    
        if run_status:
            if run_status == 3:                
                sd_state = decide_state(sd_state, visual_objects)
                pass
                
            elif run_status == 2:
                countdown_to_run(runmode[1])
                
            elif run_status == 7:
                rospy.signal_shutdown('node has been killed')
                sys.exit()
    
        state_pub.publish(sd_state)
        status_pub.publish(run_status)

        rate.sleep()
