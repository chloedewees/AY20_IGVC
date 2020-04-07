#!/usr/bin/env python
#import the necessary packages (shapely)
#this whole thing is incredibly convoluted and should be burned
#it works well if both lanes are visible and there isn't a horizontal intersection that messes with the right lane
import rospy
import itertools
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union, transform
bridge = CvBridge()

imgWidth = 1920
imgHeight = 1440
#cropped dims are 1536 x 720
cv_image = np.zeros((4*imgWidth/5,imgHeight/2,3), np.uint8)
res = np.zeros((4*imgWidth/5,imgHeight/2,3), np.uint8)
image_size = (4*imgWidth/5,imgHeight/2)
numSectors = 4

leftDetected = False
lastLeftLine = None

# filter values for white lanes
hul=0
huh=20
sal=0
sah=20
val=235
vah=255

HSVLOW=np.array([hul,sal,val])
HSVHIGH=np.array([huh,sah,vah])

pub = rospy.Publisher('/y_distance',Float32,queue_size=1)
pub_image_top_down = rospy.Publisher('lane_detection_image_top_down',Image,queue_size=1)
pub_image_seek = rospy.Publisher('lane_detection_image_seek',Image,queue_size=1)

#set up sector limits
#wInt = int(1536/numSectors)
#hInt = int(720/numSectors)
#polyList = []

#for y in range (0,numSectors):
#    for x in range(0,numSectors):
#        newPoly = Polygon([(x*wInt,y*hInt),((x+1)*wInt,y*hInt),((x+1)*wInt,(y+1)*hInt),(x*wInt,(y+1)*hInt)])
#        polyList.append(newPoly)

# these sectors are use with the contour polygons
# to sort our possible lanes between the left and right
bottomHalf = Polygon([(0,360),(1536,360),(1536,720),(0,720)])
bottomBorder = Polygon([(0,715),(1536,715),(1536,720),(0,720)])
bottomLeft = Polygon([(0,480),(768,480),(768,720),(0,720)])
bottomRight = Polygon([(768,480),(1536,480),(1536,720),(768,720)])

def unwarp_lane(img):
    undist = img
    undist_orig = undist.copy()
    # this aims to transform the forward camera input to a more top-down view
    # but it's not perfect
    angle_rad = 55*3.1415/180
    length = 800
    x1 = 0
    y1 = 720
    x2 = x1+int(length*math.cos(angle_rad))
    y2 = y1-int(length*math.sin(angle_rad))
    x4 = 1536
    y4 = 720
    x3 = x4-int(length*math.cos(angle_rad))
    y3 = y4-int(length*math.sin(angle_rad))

    cv2.line(undist_orig, (x1,y1), (x2, y2), [255,0,0], 5)
    cv2.line(undist_orig, (x3,y3), (x4, y4), [255,0,0], 5)

    img = bridge.cv2_to_imgmsg(undist_orig,'bgr8')
    pub_image_seek.publish(img)

    src = np.float32([(x1,y1),(x2,y2),(x3,y3),(x4,y4)])

    img_size = image_size
    dst = np.float32([(x1,y1),(x1,0),(x4,0),(x4,y4)])
    M = cv2.getPerspectiveTransform(src, dst)

    warped = cv2.warpPerspective(undist, M, img_size)

    img = bridge.cv2_to_imgmsg(warped,'bgr8')
    pub_image_top_down.publish(img)

    global top_right
    top_right = undist_orig

    global bottom_right
    bottom_right = warped
    
    return warped, M

def find_lane(polyArray, contourArray, image):
    global leftDetected, lastLeftLine
    # look for polygons in bottom half to find lane starts (or third?)
    # we are zipping together our polygons and contours to keep them together
    allElemsIter = iter(sorted(list(zip(polyArray, contourArray)), key = lambda p: -p[0].bounds[3]))
    goodElems = [x.intersects(bottomHalf) for x in polyArray]
    filteredPolys = list(itertools.compress(polyArray, goodElems))
    filteredContours = list(itertools.compress(contourArray, goodElems))
    
    # sort possible lane parts by largest y value (closest to vehicle)
    # we do this because there are often extra lanes that we don't care about that are further away from the vehicle
    leftElems = [x.intersects(bottomLeft) for x in filteredPolys]
    filteredLeftPolys = list(itertools.compress(filteredPolys, leftElems))
    filteredLeftContours = list(itertools.compress(filteredContours, leftElems))
    lefts = sorted(list(zip(filteredLeftPolys, filteredLeftContours)), key = lambda p: -p[0].bounds[3])
    
    #sort by closest to bottom, then closest to center
    rightElems = [x.intersects(bottomRight) for x in filteredPolys]
    filteredRightPolys = list(itertools.compress(filteredPolys, rightElems))
    filteredRightContours = list(itertools.compress(filteredContours, rightElems))
    rights = sorted(sorted(list(zip(filteredRightPolys, filteredRightContours)), key = lambda p: -p[0].bounds[3]), key = lambda p: p[0].bounds[2])

    leftLane = []
    rightLane = []
    # build left lane from sorting bottom to top
    nextElem = lefts[0] if 0 < len(lefts) else None
    i = 0
	
    while nextElem != None:
	#  add our next piece to our left lane
        x,y = nextElem[0].exterior.coords.xy
        leftLane.append((x,y))
	# color it in
        cv2.fillConvexPoly(image, np.array(zip(x,y), 'int32'),(0,0,0))
	# fit line to our furthest forward left lane element
	# if our lane is sloping right/left, this tells us to look to the right/left for our next element
        [vx, vy, x, y] = cv2.fitLine(nextElem[1], cv2.DIST_L2, 0 ,0.01,0.01)
        # when our closest element reaches the bottom of the image, the slope of the line gets skewed
	# so when we detect that our first element has reached the bottom, we save its slope until it's gone completely
        if i == 0:
            if nextElem[0].bounds[3] > 710 and lastLeftLine != None:
                (vx,vy) = lastLeftLine
            else:
                lastLeftLine = (vx, vy)
        lefty = int((-x*vy/vx)+y)
        righty = int(((image.shape[1]-x)*vy/vx)+y)
        #cv2.line(image,(image.shape[1]-1,righty),(0,lefty),255,2)
	# we look up to 400 pixels out for our next lane element
        lineLen = 400
        m = vy/vx
        r = math.sqrt(1+m**2)
        x2 = int(x + lineLen/r)
        y2 = int(y + lineLen*m/r)
        #cv2.line(image,(x,y),(x2,y2),255,50)
	# we look 50 pixels wide along our 400 px length line
        seekingLine = LineString([(x,y),(x2,y2)]).buffer(50)
        # skip repeat element (we look at allElem[0] to start, don't want to do it again
        if i == 0:
            nextElem = next((x for x in allElemsIter if x[0].intersects(seekingLine)), None)
        nextElem = next((x for x in allElemsIter if x[0].intersects(seekingLine)), None)
        i += 1
        
    if len(lefts) > 0:
	# get all the coordinates of points in the left lane and fit to quadratic
        x,y = list(zip(*leftLane))
        x = [val for sub in x for val in sub]
        y = [val for sub in y for val in sub]
        #cv2.fillConvexPoly(image, np.array(zip(x,y), 'int32'),(255,255,0))
        myCoeffs = np.polyfit(x,y,2)
        y = [myCoeffs[2] + myCoeffs[1]*x1 + myCoeffs[0]*(x1**2) for x1 in x]
        coords = np.int32([np.array(list(zip(x,y)))])
        cv2.polylines(image,coords, False, 0, thickness=3)
   
    if len(rights) > 0:
	# get all the coordinates of points in the right lane and fit to quadratic
	# because right lane is solid, we don't piece it together like the left lane
        x,y = rights[0][0].exterior.coords.xy
        cv2.fillConvexPoly(image, np.array(zip(x,y), 'int32'),(255,255,0))
        myCoeffs = np.polyfit(x,y,2)
        y = [myCoeffs[2] + myCoeffs[1]*x1 + myCoeffs[0]*(x1**2) for x1 in x]
        coords = np.int32([np.array(list(zip(x,y)))])
        cv2.polylines(image,coords, False, 0, thickness=3)

    # not sure what we want to return here

def callback(data):
    global cv_image
    global res
    boxPolyArray = []
    contourArray = []
    cv_image_orig = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    
    # crop image
    cv_image = cv_image_orig[imgHeight/2:imgHeight,imgWidth/10+225:9*imgWidth/10+225]
    cv_image=cv2.GaussianBlur(cv_image,(5,5),0)
    
    #try getting topDown image
    cv_image_top, M = unwarp_lane(cv_image)
    #cv2.imshow('top_down', cv_image_top)
    hsv=cv2.cvtColor(cv_image_top, cv2.COLOR_BGR2HSV)
    # apply mask to topDown image to find white pixels
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
    res = cv2.bitwise_and(cv_image_top,cv_image_top, mask = mask)
    # erode and dilate - reduce noise
    kernel = np.ones((6,6),np.uint8)
    res = cv2.erode(res,kernel, iterations = 1)
    res = cv2.dilate(res,kernel, iterations = 1)
    res2 = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # find counters
    _, contours, hierarchy = cv2.findContours(res2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if contours:
      for index,contour in enumerate(contours):
	# get h,w of contour bounding box to ensure it's not tiny
        rect = cv2.minAreaRect(contour)
        (center, (w, h), angle) = rect
	# get the points along the contours to create polygons
	# these polygons will check for collision with other polygons
	# in order to determine their position
        epsilon = 0.1*cv2.arcLength(contour,True)
        polyPoints = np.int0([val for sublist in contour for val in sublist])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        if(w*h >= 500):
            cv2.drawContours(cv_image_top, contour, -1, (0,255,0), 3)
            cv2.drawContours(res, contour, -1, (0,255,0), 3)
            #boxConvert = map(lambda p: tuple(p), approx)
            boxPoly = Polygon(polyPoints)
            boxPolyArray.append(boxPoly)
            contourArray.append(contour)
                    
    # Now use our polygons to find lanes
    find_lane(boxPolyArray, contourArray, cv_image_top)
    # note that we never call measure_center, not sure what input to give it.

    cv2.imshow('Camera', cv_image_top)
    #cv2.imshow('res', res)
    cv2.waitKey(1)

def measure_center(img,left_fitx,right_fitx):
    lane_width_p = right_fitx[-1] - left_fitx[-1]
    # TODO: figure out the actual value of px to meters - requires field testing
    # we found it for rightLaneDetector but it is probably different for this
    xm_per_pix = 3.7 / lane_width_p
    
    x_pixel_dim = img.shape[0]
    x_pixel_center = x_pixel_dim/2.00
    deviation_from_center = (x_pixel_center - (left_fitx[-1] + lane_width_p/2))*xm_per_pix
    
    # return lane width and camera's position
    pub.publish(float(deviation_from_center))
    return lane_width_p*xm_per_pix, deviation_from_center

def listener():
    global cv_image
    global res
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lanedetection', anonymous=True)
    rate = rospy.Rate(12) # 12Hz to sync with 12 fps camera. Increasing this might be good
    rospy.Subscriber("camera_fm/camera_fm/image_raw",Image, callback)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for Image")
        listener()
    except KeyboardInterrupt:
        print("Goodbye")
