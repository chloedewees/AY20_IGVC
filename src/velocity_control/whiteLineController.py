#!/usr/bin/env python

import rospy, sys
from std_msgs.msg import Int16, Int8, Float32
from pacmod_msgs.msg import PacmodCmd

brake_cmd = PacmodCmd()
throttle_cmd = PacmodCmd()

state = 0

VC = [50.0, 75.0]
C = [50.0, 75.0, 100.0]
M = [75.0, 150.0, 200.0]
F = [150.0, 200.0, 250.0]
VF = [200.0, 250.0]

deltaST = [1.0, 2.0]
deltaVS = [1.0, 2.0, 5.0]
deltaS = [2.0, 5.0, 10.0]
deltaF = [5.0, 10.0, 15.0]
deltaVF = [15.0, 20.0]

NOSPEED = 0.0
QUARTERSPEED = 1.25
HALFSPEED = 2.5
FULLSPEED = 5.0

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
[NOSPEED, NOSPEED, NOSPEED, NOSPEED, NOSPEED],
[HALFSPEED, QUARTERSPEED, NOSPEED, NOSPEED, NOSPEED],
[HALFSPEED, HALFSPEED, QUARTERSPEED, NOSPEED, NOSPEED],
[FULLSPEED, FULLSPEED, HALFSPEED, QUARTERSPEED, QUARTERSPEED],
[FULLSPEED, FULLSPEED, FULLSPEED, FULLSPEED, HALFSPEED]
]

speedOutput = 0.0
yDist = 0.0
prevYDist = 0.0
deltaYCoord = 2.0

def state_callback(msg):
    global state
    state = msg.data
    if state == 7:
        rospy.signal_shutdown('killed by selfdrive manager')
        sys.exit()


def callback(data):
    global yDist, prevYDist, deltaYCoord
    prevYDist = yDist
    if data.data == -1:
        yDist = 1000
    else:
        yDist = data.data
    deltaYCoord = abs(yDist - prevYDist)

def whiteLineSpeedController():
    global speedOutput, yDist, brake_cmd, throttle_cmd
    rospy.init_node('whiteLineBreakController', anonymous=True)
    throttle_pub = rospy.Publisher('pacmod/as_rx/accel_cmd', PacmodCmd, queue_size = 10)
    speed_pub = rospy.Publisher('/speed_applied', Float32, queue_size=10)
    rospy.Subscriber('/selfdrive/state', Int8, state_callback)
    rospy.Subscriber('/line_distance', Int16, callback)   
    
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if state == 11:
            throttle_cmd.f64_cmd = 0.0
            throttle_pub.publish(throttle_cmd)
            loop()
            print(speedOutput)
            speed_cmd = speedOutput
            speed_pub.publish(speed_cmd)
            rate.sleep()
        
    
def loop():
    global speedOutput, membership, deltaMembership, yDist, deltaYCoord
    getState()
    getMembership(yDist, deltaYCoord)
    combVectors(membership, deltaMembership)
    print(membership)
    speedOutput = defuzzification()


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
    global yDist, prevYDist, deltaYCoord
    prevYDist = yDist
    deltaYCoord = yDist - prevYDist

def getMembership(x, y):
    global membership, deltaMembership, VC, C, M, F, VF, deltaST, deltaVS, deltaS, deltaF, deltaVF
    membership[0] = fuzzificationTrapizoid(x, VC[0], VC[1], "left")
    membership[1] = fuzzificationTriangle(x, C[0], C[1], C[2])
    membership[2] = fuzzificationTriangle(x, M[0], M[1], M[2])
    membership[3] = fuzzificationTriangle(x, F[0], F[1], F[2])
    membership[4] = fuzzificationTrapizoid(x, VF[0], VF[1], "right")

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
    
    whiteLineSpeedController()
