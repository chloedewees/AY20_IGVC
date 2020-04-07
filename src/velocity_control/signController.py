#!/usr/bin/env python

import rospy, sys
from std_msgs.msg import UInt16MultiArray, Int8
from pacmod_msgs.msg import PacmodCmd

brake_cmd = PacmodCmd()
throttle_cmd = PacmodCmd()

state = 0

VC = [2500.0, 3000.0]
C = [1500.0, 2000.0, 3000.0]
M = [1400.0, 1500.0, 2000.0]
F = [1200.0, 1400.0, 1500.0]
VF = [1200.0, 1400.0]
deltaST = [50.0, 100.0]
deltaVS = [50.0, 100.0, 200.0]
deltaS = [100.0, 200.0, 300.0]
deltaF = [200.0, 300.0, 500.0]
deltaVF = [300.0,   500.0, 1000.0]
NOBREAK = 0.4
QUARTERBREAK = 0.625
HALFBREAK = 0.75
FULLBREAK = 1.0

membership = [0.0, 0.0, 0.0, 0.0, 0.0]
deltaMembership = [0.0, 0.0, 0.0, 0.0, 0.0]

rows = 5
collums = 5

degreeFiring = [
[0,0,0,0,0],
[0,0,0,0,0],
[0,0,0,0,0],
[0,0,0,0,0],
[0,0,0,0,0]
]

outputTable = [
[FULLBREAK, FULLBREAK, FULLBREAK, FULLBREAK, FULLBREAK],
[NOBREAK, QUARTERBREAK, HALFBREAK, FULLBREAK, FULLBREAK],
[NOBREAK, NOBREAK, QUARTERBREAK, HALFBREAK, FULLBREAK],
[NOBREAK, NOBREAK, NOBREAK, QUARTERBREAK, HALFBREAK],
[NOBREAK, NOBREAK, NOBREAK, NOBREAK, QUARTERBREAK]
]

breakOutput = 0.0
area = 0.0
prevArea = 0.0
deltaArea = 2.0

def state_callback(msg):
    global state
    state = msg.data
    if state == 7:
        rospy.signal_shutdown('killed by selfdrive manager')
        sys.exit()

def callback(data):
    global area, prevArea, deltaArea
    prevArea = area
    if (len(data.data) == 0):
        area = 0
    else:
        area = data.data[0]
    deltaArea = abs(area - prevArea)

def stopSignBreakController():
    global breakOutput, area, brake_cmd, throttle_cmd
    rospy.init_node('stopSignBreakController', anonymous=True)
    brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=10)
    throttle_pub = rospy.Publisher('pacmod/as_rx/accel_cmd', PacmodCmd, queue_size = 10)
    rospy.Subscriber('/selfdrive/state', Int8, state_callback)
    rospy.Subscriber("detection_status", UInt16MultiArray, callback)   
    
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if state == 11:
            throttle_cmd.f64_cmd = 0.0
            throttle_pub.publish(throttle_cmd)
            loop()
            #print(breakOutput)
            brake_cmd.f64_cmd = breakOutput
            brake_pub.publish(brake_cmd)
        rate.sleep()
        
    
def loop():
    global breakOutput, membership, deltaMembership, area, deltaArea
    getState()
    getMembership(area, deltaArea)
    combVectors(membership, deltaMembership)

    breakOutput = defuzzification()


def fuzzificationTriangle(x, p1, p2, p3):
    if x < p1 or x > p3:
        return 0.0
    elif x >= p1 and x <= p2:
        slope = 1/(p2-p1)
        b = 0-(slope*p1)
        return (x*slope) + b
    else:
        slope = 1/(p3-p2)
        b = 0-(-slope*p3)
        return (x*(-slope)) + b

def fuzzificationTrapizoid(x, p1, p2, end):
    if x < p1 and end == "left":
        return 1.0
    elif x < p1 and end == "right":
        return 0.0
    elif x >= p1 and x <= p2:
        slope = 1/(p2-p1)
        if end == "left":
          b = 0-(-slope*p2)
          return (x*(-slope)) + b
        else:
          b = 0-(slope*p1)
          return (x*slope) + b
    elif x > p2 and end == "left":
        return 0.0
    else:
        return 1.0

def getState():
    global area, prevArea, deltaArea
    prevArea = area
    deltaArea = area - prevArea

def getMembership(x, y):
    global membership, deltaMembership, VC, C, M, F, VF, deltaST, deltaVS, deltaS, deltaF, deltaVF
    membership[0] = fuzzificationTrapizoid(x, VC[0], VC[1], "right")
    membership[1] = fuzzificationTriangle(x, C[0], C[1], C[2])
    membership[2] = fuzzificationTriangle(x, M[0], M[1], M[2])
    membership[3] = fuzzificationTriangle(x, F[0], F[1], F[2])
    membership[4] = fuzzificationTrapizoid(x, VF[0], VF[1], "left")

    deltaMembership[0] = fuzzificationTrapizoid(y, deltaST[0], deltaST[1], "left")
    deltaMembership[1] = fuzzificationTriangle(y, deltaVS[0], deltaVS[1], deltaVS[2])
    deltaMembership[2] = fuzzificationTriangle(y, deltaS[0], deltaS[1], deltaS[2])
    deltaMembership[3] = fuzzificationTriangle(y, deltaF[0], deltaF[1], deltaF[2])
    deltaMembership[4] = fuzzificationTrapizoid(y, deltaVF[0], deltaVF[1], "right")

def combVectors(vec1, vec2):
    global degreeFiring, collums, rows
    for i in range(collums):
        for j in range(rows):
            degreeFiring[i][j]=vec1[i]*vec2[j]

def defuzzification(): 
    global collums, rows, degreeFiring, outputTable
    runningNumSum = 0
    runningDenSum = 0
    for i in range(collums):
        for j in range(rows):
            runningNumSum += (degreeFiring[i][j]*outputTable[i][j])
            runningDenSum += (degreeFiring[i][j])
    return runningNumSum/runningDenSum
    
if __name__ == '__main__':
    stopSignBreakController()
