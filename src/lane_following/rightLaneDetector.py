#!/usr/bin/env python
#import the necessary packages
#this used to be colorFind.py
import rospy
import itertools
from sensor_msgs.msg import Image
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
cv_image = np.zeros((imgWidth, imgHeight,3), np.uint8)
res = np.zeros((imgWidth, imgHeight,3), np.uint8)
image_size = (imgWidth, imgHeight)

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
pub_image_top_down = rospy.Publisher('/lane_detection_image_top_down',Image,queue_size=1)

def unwarp_lane(img):
    # this transforms the input from the right camera to a more straight-ahead view of the right lane
    undist = img
    undist_orig = undist.copy()
    
    rad = 1.26
    length = 800
    x1 = 714
    y1 = 1429
    x2 = x1-int(length*math.cos(rad))
    y2 = y1-int(length*math.sin(rad))
    x4 = 1478
    y4 = 1185
    x3 = x4-int(length*math.cos(rad))
    y3 = y4-int(length*math.sin(rad))

    cv2.line(undist_orig, (x1,y1), (x2, y2), [255,0,0], 5)
    cv2.line(undist_orig, (x3,y3), (x4, y4), [255,0,0], 5)

    img = bridge.cv2_to_imgmsg(undist_orig,'bgr8')

    src = np.float32([(x1,y1),(x2,y2),(x3,y3),(x4,y4)])


    dst = np.float32([(0,800),(0,0),(800,0),(800,800)])
    M = cv2.getPerspectiveTransform(src, dst)
    # we warp the perspective and then crop out the 800x800 piece
    warped = cv2.warpPerspective(undist_orig, M, image_size)
    cropped = warped[0:800,0:800]
    img = bridge.cv2_to_imgmsg(cropped,'bgr8')
    return cropped, M


def find_lane(polyArray, contourArray, image):
    
    # sort by closest to bottom, then closest to center (this might be wrong / backwards)
    # the zipping keeps the polygon and contours together, now unnecessary
    rights = sorted(sorted(list(zip(polyArray, contourArray)), key = lambda p: p[0].bounds[2]), key = lambda p: -p[0].bounds[3])
    
    coords = None
    if len(rights) > 0: # do we have any possibilities for the lane?
        coords = rights[0][0].exterior.coords.xy #get closest right lane
    	lane = list(zip(coords[0],coords[1]))
    else:
	lane = None

    return lane # returns the polygon x,y coordinates of the right lane (if it exists)

def callback(data):
    global cv_image
    global res
    boxPolyArray = []
    contourArray = []
    cv_image_orig = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    cv_image = cv_image_orig[:]
    cv_image=cv2.GaussianBlur(cv_image,(5,5),0)
    
    # transform into better aligned image
    cv_image_top, M = unwarp_lane(cv_image)
    hsv=cv2.cvtColor(cv_image_top, cv2.COLOR_BGR2HSV)
    # apply white pixel mask to transformed image, will want to make this more robust by possibily looking at edge detection instead of set HSVs values
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
    res = cv2.bitwise_and(cv_image_top,cv_image_top, mask = mask)
    # erode and dilate to reduce noise
    kernel = np.ones((6,6),np.uint8)
    res = cv2.erode(res,kernel, iterations = 1)
    res = cv2.dilate(res,kernel, iterations = 1)
    res2 = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # find counters of white pixels
    _, contours, hierarchy = cv2.findContours(res2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if contours:
      # draw contours and create polygons that are >= 500 pixels
      for index,contour in enumerate(contours):
	# get h,w of contour bounding box for minimum size
        rect = cv2.minAreaRect(contour)
        (center, (w, h), angle) = rect
	# find points that fit along the contours to create polygons
        epsilon = 0.1*cv2.arcLength(contour,True)
        polyPoints = np.int0([val for sublist in contour for val in sublist])
        if(w*h >= 9000): # ignore tiny bounding boxes
            cv2.drawContours(cv_image_top, contour, -1, (0,255,0), 3)
            cv2.drawContours(res, contour, -1, (0,255,0), 3)
            #boxConvert = map(lambda p: tuple(p), approx)
            boxPoly = Polygon(polyPoints)
            boxPolyArray.append(boxPoly)
            contourArray.append(contour)
                    
    # Now use our polygons to find lanes
    lane = find_lane(boxPolyArray, contourArray, cv_image_top)
    # and measure how far away we are from the center
    dev, cv_image_top = measure_center(lane, cv_image_top)

    final_img = bridge.cv2_to_imgmsg(cv_image_top,'bgr8')
    pub_image_top_down.publish(final_img)
    cv2.waitKey(1)

def measure_center(lane, image):
		# On undistorted image, 1 ft = 200px - 0.06 in / px
		# Distorted maps 800 px to 764
		# 1ft = 0.3048m
		# it's 0.1524 cm to px
		# car is 55 in across, lane is 96 in.
		# this gives up 41/2 = 20.5 in to be centered

		m_per_pix = 0.001524
		x_distance_centered = 200 * (20.5/12) # centered if 20.5 in away
		lane_width_m = 2.4384

		right_lane = lane
		if right_lane != None:
			lane_vector = np.array([right_lane]) #convert to numpy array for line fitting

			# Invert x and y so that it fits to vertical lines (see what happens if you switch them)
			# uses a quadratic fit to help with curves as well
			x,y = list(zip(*right_lane))
			myCoeffs = np.polyfit(y,x,2)
			x = [myCoeffs[2] + myCoeffs[1]*y1 + myCoeffs[0]*y1**2 for y1 in y]
			coords = np.int32([np.array(list(zip(x,y)))]) # but we want x,y plotted as such
			cv2.polylines(image,coords, False, 0, thickness=3)
			intersection = False # did we find a more accurate point?
			distance = -1
			for y in range(-800,1601): # find perp slope that goes through (0,800)
				perp_slope = -1/(2*myCoeffs[0]*y+myCoeffs[1]) # we are still inverting x,y
				x = myCoeffs[2] + myCoeffs[1]*y + myCoeffs[0]*y**2
				b = -perp_slope*y+x
				if(abs(perp_slope*800+b) <= 15): # have we found our true distance to the line?
					intersection = True
					truex,truey = x,y
					distance = math.sqrt(x**2+(y-800)**2)
					cv2.line(image, (0,800), (int(truex),int(truey)), [0,0,255], 3)
					break
			
			if intersection:
				right_x = distance - 30 # subtract 30 because we want lane tape, not the fit line
				#print(distance)
				deviation_from_center = -(x_distance_centered - right_x)*m_per_pix 		# negative deviation if car is right of center
			else: # if we can't find the actual x, just default to bottom of image
				right_lane = filter(lambda p: 800-p[1]<20, right_lane)
				right_lane = sorted(right_lane, key = lambda p: p[0])
				if right_lane != None:
					right_x = right_lane[0][0]
					deviation_from_center = -(x_distance_centered - right_x)*m_per_pix # negative deviation if car is right of center
				else: # no lanes, we screwed
					deviation_from_center = -99

		else: # no lanes, we screwed
			deviation_from_center = -99

		# return lane width and camera's position
		#rospy.loginfo(deviation_from_center)
		pub.publish(float(deviation_from_center))
		return deviation_from_center, image

def listener():
    global cv_image
    global res
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('right_lane_detection', anonymous=True)
    rate = rospy.Rate(12) # 12hz - camera runs at 12 fps. Increasing this might be good
    rospy.Subscriber("camera_fr/camera_fr/image_raw",Image, callback)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for Image")
        listener()
    except KeyboardInterrupt:
        print("Goodbye")
