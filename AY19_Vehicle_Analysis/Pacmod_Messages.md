# Pacmod analysis of the bagfile  
gem_e2_2019-06-21-11-26-44.bag
- Note: During this run the vehicle was driven manually by use of steering wheel, gas pedal and brake pedal. We did not expect commands to be sent to the can bus we only expected to recieve data from it.

## This is a list of message IDs found being published to topic /pacmod/can_tx  
TX Set([100, 102, 104, 106, 108, 110, 111, 112, 113, 114, 115, 116, 117, 255])

## These numbers map to:  
Referencing:   
https://github.com/astuff/pacmod1_2_dbc/blob/master/as_pacmod_v2.dbc  

TURN_RPT, SHIFT_RPT, ACCEL_RPT, GLOBAL_RPT, BRAKE_RPT, STEERING_RPT_1, VEHICLE_SPEED_RPT, BRAKE_MOTOR_RPT_1, BRAKE_MOTOR_RPT_2, BRAKE_MOTOR_RPT_3, STEERING_MOTOR_RPT_1, STEERING_MOTOR_RPT_2, STEERING_MOTOR_RPT_3  

Message ID: 255 is not found in this reference.
 
## This data is published in the following topics.

|ROSTOPIC|CAN MSG|ID|
|--------|-------|--|
|/pacmod/parsed_tx/turn_rpt|TURN_RPT|100|
|/pacmod/parsed_tx/shift_rpt|SHIFT_RPT|102|
|/pacmod/parsed_tx/accel_rpt|ACCEL_RPT|104|
|/pacmod/parsed_tx/global_rpt|GLOBAL_RPT|106|
|/pacmod/parsed_tx/brake_rpt|BRAKE_RPT|108|
|/pacmod/parsed_tx/steer_rpt|STEERING_RPT_1|110|
|/pacmod/parsed_tx/vehicle_speed_rpt|VEHICLE_SPEED_RPT|111|
|/pacmod/parsed_tx/brake_rpt_detail_1|BRAKE_MOTOR_RPT_1|112|
|/pacmod/parsed_tx/brake_rpt_detail_2|BRAKE_MOTOR_RPT_2|113|
|/pacmod/parsed_tx/brake_rpt_detail_3|BRAKE_MOTOR_RPT_3|114|
|/pacmod/parsed_tx/steer_rpt_detail_1|STEERING_MOTOR_RPT_1|115|
|/pacmod/parsed_tx/steer_rpt_detail_2|STEERING_MOTOR_RPT_2|116|
|/pacmod/parsed_tx/steer_rpt_detail_3|STEERING_MOTOR_RPT_3|117|

## These topics published pacmod do not appear to map directly to the above messages
- /pacmod/as_tx/enable  --> This topic publishes a single bool value. For this run it remained false.
- /pacmod/as_tx/vehicle_speed  --> This topic appears to publish vehicle speed in m/s.
- /pacmod/parsed_tx/vin_rpt  --> This topic published the below topic.  
&nbsp;&nbsp;header:   
&nbsp;&nbsp;&nbsp;&nbsp;  seq: 2  
&nbsp;&nbsp;&nbsp;&nbsp;  stamp:   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    secs: 1561130805  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    nsecs: 430815084  
&nbsp;&nbsp;&nbsp;&nbsp;  frame_id: "pacmod"  
&nbsp;&nbsp;mfg_code: "52c"  
&nbsp;&nbsp;mfg: ''  
&nbsp;&nbsp;model_year_code: "j"  
&nbsp;&nbsp;model_year: 0  
&nbsp;&nbsp;serial: 19216  

## The message ID 255 is not accounted for in reference document.
Below is the /pacmod/can_tx message published with id of 255. These values do not appear to change at all.

header:   
&nbsp;&nbsp; seq: 33129  
&nbsp;&nbsp; stamp:   
&nbsp;&nbsp;&nbsp;&nbsp;  secs: 1561130888  
&nbsp;&nbsp;&nbsp;&nbsp;  nsecs: 769860832  
&nbsp;&nbsp; frame_id: "0"  
id: 255  
is_rtr: False  
is_extended: False  
is_error: False  
dlc: 8  
data: [53, 50, 99, 106, 0, 75, 16, 0]  


