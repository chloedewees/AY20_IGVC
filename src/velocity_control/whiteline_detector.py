#!/usr/bin/env python
#import the necessary packages
import rospy, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import cv2
import numpy as np
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

last_detected = False
ourGuy = -1

Help_Save_Me_From_this_eternal_torment = rospy.Publisher('/line_distance', Int16, queue_size=1)
pub_image_top_down = rospy.Publisher('line_detection_image_top_down',Image,queue_size=1)


#set up sector limits
wInt = int(1536/numSectors)
hInt = int(720/numSectors)
polyList = []

# these polygon sectors will be used to see if we have a horizontal white line going all the way across the image
for y in range (0,numSectors):
    for x in range(0,numSectors):
        newPoly = Polygon([(x*wInt,y*hInt),((x+1)*wInt,y*hInt),((x+1)*wInt,(y+1)*hInt),(x*wInt,(y+1)*hInt)])
        polyList.append(newPoly)

# filter values for white lanes
hul=0
huh=20
sal=0
sah=20
val=235
vah=255

HSVLOW=np.array([hul,sal,val])
HSVHIGH=np.array([huh,sah,vah])

def unwarp_lane(img):
    # get top/down view of lane
    undist = img
    undist_orig = undist.copy()

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
    
    #print(warped.shape[:2])
    return warped, M

def callback(data):
    global cv_image
    global res
    global last_detected
    global ourGuy

    cv_image_orig = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    
    # crop image
    # we should try cropping more, because the image is very wide as is
    cv_image = cv_image_orig[imgHeight/2:imgHeight,imgWidth/10:9*imgWidth/10]
    cv_image=cv2.GaussianBlur(cv_image,(5,5),0)
    
    # getting topDown image
    cv_image_top, M = unwarp_lane(cv_image)
    #cv2.imshow('top_down', cv_image_top)
    hsv=cv2.cvtColor(cv_image_top, cv2.COLOR_BGR2HSV)
    # apply this to the top/down thing
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
    res = cv2.bitwise_and(cv_image_top,cv_image_top, mask = mask)
    # erode and dilate
    kernel = np.ones((6,6),np.uint8)
    res = cv2.erode(res,kernel, iterations = 1)
    res = cv2.dilate(res,kernel, iterations = 1)
    res2 = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # find counters
    _, contours, hierarchy = cv2.findContours(res2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if contours:
      # draw contours and create polygons that are >= 500 pixels
      for index,contour in enumerate(contours):
        rect = cv2.minAreaRect(contour)
        (center, (w, h), angle) = rect
        epsilon = 0.1*cv2.arcLength(contour,True)
        polyPoints = np.int0([val for sublist in contour for val in sublist])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        if(w*h >= 500):
            cv2.drawContours(cv_image_top, contour, -1, (0,255,0), 3)
            cv2.drawContours(res, contour, -1, (0,255,0), 3)
            #boxConvert = map(lambda p: tuple(p), approx)
            myPoly = Polygon(polyPoints) # convert to polygon to check collision
            line_detected = 0
            oldGuy = ourGuy
            ourGuy = -1
            for i in range (0,numSectors**2):
                # Do we have a solid white line in front of us? - look to see if single line crosses all the way across
                if(myPoly.intersects(polyList[12]) and myPoly.intersects(polyList[15])) or (myPoly.intersects(polyList[8]) and myPoly.intersects(polyList[11])):
                    line_detected = 1
                    last_detected = True
                    (x,y) = myPoly.exterior.coords.xy
                    coord_list = list(zip(x,y))
                    filtered_coords = list(filter(lambda p: p[0] == 768,coord_list)) #768 is down the middle of the image
                    # ourGuy is pixel distance from bottom to white line
                    ourGuy = image_size[1]-max(filtered_coords, key=lambda p: p[1])[1]
                else: # we use last_detected because even if there is a white line, every other frame comes up as "false"
		    # this gives us a 1 frame buffer to stop getting true, false, true, false, etc.
                    if last_detected == True:
                        last_detected = False
                        line_detected = 1
                        ourGuy = oldGuy
            rospy.loginfo(ourGuy)
                    
    h1, w1, _ = cv_image_top.shape
    #this just draws the sectors, pretty unnecessary
    h1 = int(h1)
    w1 = int(w1)
    wP = int(w1/numSectors)
    hP = int(h1/numSectors)
    for i in range (1,numSectors):
        cv2.line(cv_image_top,(wP*i,0),(wP*i,h1),(50,50,255),3)
        cv2.line(cv_image_top,(0,hP*i),(w1,hP*i),(50,50,255),3)

    cv2.imshow('Camera', cv_image_top)
    #cv2.imshow('res', res)
    cv2.waitKey(1)


def listener():
    global cv_image, res, ourGuy
  
    rospy.init_node('linedetection',anonymous=True)
    rate = rospy.Rate(12) # 12hz
    topic = "/camera_fm/camera_fm/image_raw"
    rospy.Subscriber(topic, Image, callback)
    while not rospy.is_shutdown():
    # publish pixel distance from car to line; -1 if not found
        Help_Save_Me_From_this_eternal_torment.publish(ourGuy)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        print("Goodbye")
