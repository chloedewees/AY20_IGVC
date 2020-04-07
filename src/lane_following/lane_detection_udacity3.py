#!/usr/bin/env python
import roslib,rospy,sys,cv2,time
import numpy as np
import math
#roslib.load_manifest('lane_follower')
# from __future__ import print_function
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError

# Udacity
import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.pyplot
import copy
import time
import math

# Define a class to receive the characteristics of each line detection
class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = None     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        #polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]  
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float') 
        #x values for detected line pixels
        self.allx = None
        #y values for detected line pixels
        self.ally = None

        self.ploty = None

# initialize line structs
global left
global right
#global frame
left = Line()
right = Line()

# change image size here, height x width (sike)
image_size = (1920,1440)

bridge = CvBridge()
pub = rospy.Publisher('lane_detection', Int32, queue_size=10) #ros-lane-detection
pub_image = rospy.Publisher('lane_detection_image',Image,queue_size=1)
pub_image_top_down = rospy.Publisher('lane_detection_image_top_down',Image,queue_size=1)
pub_image_seek = rospy.Publisher('lane_detection_image_seek',Image,queue_size=1)
pub_image_quad = rospy.Publisher('lane_detection_image_quad',Image,queue_size=1)
pub_point_cloud_left = rospy.Publisher('lane_detection_point_cloud_left',PointCloud,queue_size=1)
pub_point_cloud_right = rospy.Publisher('lane_detection_point_cloud_right',PointCloud,queue_size=1)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
#objpoints = [] # 3d points in real world space
#imgpoints = [] # 2d points in image plane.

# calibration image
#img_orig = cv2.imread('4c.jpg')
#x = datetime.datetime.now()
#cv2.imwrite( "pics/output_images/test"+str(x)+".jpg", img_orig);
#img = copy.copy(img_orig)
#undist = copy.copy(img_orig)
#nx = 8 # the number of inside corners in x
#ny = 6 # the number of inside corners in y

# Find the chessboard corners
#gray = cv2.cvtColor(undist,cv2.COLOR_BGR2GRAY) 
#ret, corners = cv2.findChessboardCorners(gray, (nx,ny),None)
#print(ret)
#print(corners)

# If found, add object points, image points
#if ret == True:
#    objpoints.append(objp)
#    imgpoints.append(corners)

# calibrate camera
#et2,mtx,dist,rvecs,tvecs=cv2.calibrateCamera(objpoints, imgpoints, img.shape[1::-1],None,None)
mtx = np.matrix('1044.086264 0.000000 1051.047966;0.000000 1045.869711 762.084868;0.000000 0.000000 1.000000')
dist = np.matrix('-0.184063 0.074083 0.005065 -0.002716 0.000000')
#undist = cv2.undistort(img, mtx, dist, None, mtx)
 
#cv2.drawChessboardCorners(img_orig, (nx,ny), corners, ret)

#src = np.float32([corners[0], corners[nx-1], corners[-1], corners[-nx]])
#offset = 100
#img_size = (gray.shape[1], gray.shape[0])
#dst = np.float32([[offset, offset], [img_size[0]-offset, offset], 
#                             [img_size[0]-offset, img_size[1]-offset], 
#                             [offset, img_size[1]-offset]])
#M = cv2.getPerspectiveTransform(src, dst)
#warped = cv2.warpPerspective(undist, M, img_size)


def unwarp(img, mtx, dist):
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    undist = img
    src = np.float32([corners[0], corners[nx-1], corners[-1], corners[-nx]])
    offset = 100
    img_size = (gray.shape[1], gray.shape[0])
    dst = np.float32([[offset, offset], [img_size[0]-offset, offset], 
                             [img_size[0]-offset, img_size[1]-offset], 
                             [offset, img_size[1]-offset]])
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(undist, M, img_size)
    return warped



def hls_select(img, thresh=(0, 255)):
    # 1) Convert to HLS color space
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    # 2) Apply a threshold to the S channel
    s_channel = hls[:,:,2]
    binary_output = np.zeros_like(s_channel)
    binary_output[(s_channel > thresh[0]) & (s_channel <= thresh[1])] = 1
    # 3) Return a binary image of threshold result
    return binary_output

def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # 2) Take the derivative in x or y given orient = 'x' or 'y'
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize = sobel_kernel))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize = sobel_kernel))
    # 3) Take the absolute value of the derivative or gradient
    # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    # 5) Create a mask of 1's where the scaled gradient magnitude 
            # is > thresh_min and < thresh_max
    # 6) Return this mask as your binary_output image
    binary_output = np.zeros_like(scaled_sobel)
    binary_output[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return binary_output

def mag_thresh(img, sobel_kernel=3, mag_thresh=(0, 255)):
    
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # 2) Take the gradient in x and y separately
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # 3) Calculate the magnitude 
    gradmag = np.sqrt(sobelx**2 + sobely**2)
    # 4) Scale to 8-bit (0 - 255) and convert to type = np.uint8
    scale_factor = np.max(gradmag)/255 
    gradmag = (gradmag/scale_factor).astype(np.uint8) 
    # 5) Create a binary mask where mag thresholds are met
    # 6) Return this mask as your binary_output image
    binary_output = np.zeros_like(gradmag)
    binary_output[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1
    return binary_output

def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # 2) Take the gradient in x and y separately
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # 3) Take the absolute value of the x and y gradients
    # 4) Use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient 
    # 5) Create a binary mask where direction thresholds are met
    # 6) Return this mask as your binary_output image
    absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    binary_output =  np.zeros_like(absgraddir)
    binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1
    return binary_output


def explore(img, s_thresh=(170, 255), sx_thresh=(20, 100)):
    img = np.copy(img)
    ksize = 3 # Choose a larger odd number to smooth gradient measurements

    # Apply each of the thresholding functions
    gradx = abs_sobel_thresh(img, orient='x', sobel_kernel=ksize, thresh=(20, 100))
    grady = abs_sobel_thresh(img, orient='y', sobel_kernel=ksize, thresh=(20, 100))
    mag_binary = mag_thresh(img, sobel_kernel=ksize, mag_thresh=(30, 100))
    dir_binary = dir_threshold(img, sobel_kernel=ksize, thresh=(1, 1.3))
    
    combined = np.zeros_like(dir_binary)
    combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1

    return gradx,grady,mag_binary,dir_binary,combined 


def pipeline(img, s_thresh=(170, 255), sx_thresh=(20, 100)):
    img = np.copy(img)
    #print(img)
    #img = np.asarray(img)
    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    # Sobel x
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    # Stack each channel
    color_binary = np.dstack(( np.zeros_like(sxbinary), sxbinary, s_binary)) * 255

    # combine the color binary to black and white
    combined = np.dstack((np.max(color_binary, axis = 2), np.max(color_binary, axis = 2), np.max(color_binary, axis = 2)))
    

    global bottom_left
    bottom_left = combined

    return color_binary, combined

# define unwarp function to top-down view of lane lines
def unwarp_lane(img, mtx, dist):
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    undist = img
    undist_orig = undist.copy()
    
    offset = 400
    offset2 = 200
    angle = 45
    angle_rad = 45*3.1415/180
    length = 1000

    x1 = 190
    y1 = 1440
    x2 = x1+int(length*math.cos(angle_rad))
    y2 = y1-int(length*math.sin(angle_rad))
    x4 = 1126+offset+offset
    y4 = 1440
    x3 = x4-int(length*math.cos(angle_rad))
    y3 = y4-int(length*math.sin(angle_rad))

    # x1,y1
    #print x2,y2
    #print x3,y3
    #print x4,y4
    cv2.line(undist_orig, (x1,y1), (x2, y2), [255,0,0], 5)
    cv2.line(undist_orig, (x3,y3), (x4, y4), [255,0,0], 5)

    img = bridge.cv2_to_imgmsg(undist_orig,'bgr8')
    pub_image_seek.publish(img)

    src = np.float32([(x1,y1),(x2,y2),(x3,y3),(x4,y4)])

    #img_size = (gray.shape[1], gray.shape[0])
    img_size = image_size
    #dst = np.float32([(x1+offset2,y1),(x1+offset2,0),(x4-offset2,0),(x4-offset2,y4)])
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

def unwarp_lane2(img, mtx, dist):
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    undist = img
    #src = np.float32([corners[0], corners[nx-1], corners[-1], corners[-nx]])
    offset = 60
    offset2 = 10
    src = np.float32([(190+offset,719),(591+offset,448),(687+offset2,448),(1126+offset2,719)])
    
    img_size = (gray.shape[1], gray.shape[0])
    #dst = np.float32([[offset, offset], [img_size[0]-offset, offset], 
    #                         [img_size[0]-offset, img_size[1]-offset], 
    #                        [offset, img_size[1]-offset]])
    dst = np.float32([(200,700),(200,0),(1100,0),(1150,700)])
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(undist, M, img_size)
    return warped, M

        
def printToPic(result, K_left, K_right, lane_deviation_from_center):
    global left
    global right
    # place text on image for debugging
    # adapted from https://stackoverflow.com/questions/16615662/how-to-write-text-on-a-image-in-windows-using-python-opencv2
    if left.allx.shape[1]>1:
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        position1              = (30,100)
        position2              = (30,200)
        position3              = (30,300)
        position4              = (30,400)
        fontScale              = 2
        fontColor              = (255,165,0)
        lineType               = 5
        string = 'L/R Curvature [m]:' + str(np.around(K_left, decimals = 1)) + '/'+ str(np.around(K_right, decimals = 1))
        string2 = 'Lane Center Deviation [m]:' + str(np.around(lane_deviation_from_center, decimals = 1))
        string3 = 'Left Lane:' + str(left.detected)
        string4 = 'Right Lane:' + str(right.detected)
        #print(string)
        cv2.putText(result,string, 
        position1, 
        font, 
        fontScale,
        fontColor,
        lineType)
        
        cv2.putText(result,string2, 
        position2, 
        font, 
        fontScale,
        fontColor,
        lineType)

        cv2.putText(result,string3, 
        position3, 
        font, 
        fontScale,
        fontColor,
        lineType)
        
        cv2.putText(result,string4, 
        position4, 
        font, 
        fontScale,
        fontColor,
        lineType)

def process_image(image):
    # declare a line instance
    global left
    global left
    global right
    global frame
        
    test_img = image
    test_img = cv2.cvtColor(test_img,cv2.COLOR_BGR2RGB)
    test_img_orig = copy.copy(test_img)
    #test_img_orig = cv2.undistort(test_img_orig, mtx, dist, None, mtx)
    test_img, M = unwarp_lane(test_img,mtx,dist)
    combined, final = pipeline(test_img)
    final = cv2.cvtColor(final,cv2.COLOR_RGB2GRAY)
    final_orig = final.copy()
    out_img, ploty, left_fit_cr, right_fit_cr, left_fitx, right_fitx = fit_polynomial(final)
    right_fitx_act = np.copy(right_fitx)
    left_fitx_act = np.copy(left_fitx)

    left.ploty=ploty
    right.ploty=ploty
    
    # initialize bestx if null
    #print right.bestx
    try:
        if right.bestx == None or left.bestx == None:
            left.bestx = left_fitx
            right.bestx = right_fitx
            left.detected = True
            right.detected = True
    except ValueError:
        # Avoids an error if the above is not implemented fully
        # declare a line undetected outright if the returned shape of the curve is not 720
        pass

    if left_fitx.size != 1440:
        left.detected = False
    if right_fitx.size != 1440:
        right.detected = False
        
    # initialize the allx arays
    try:
        if left.allx == None:
            left.allx = left_fitx.reshape(1440,1)

        if right.allx == None:
            right.allx = right_fitx.reshape(1440,1)
    except ValueError:
        pass

   
    # check if lanes are resonable; if not, provide the bestx average for each lane
    # this first check is for the near side deviation
        
    offset_near_left = 200  # define offset to check for large changes 
    offset_far_left = 1000
    offset_near_right = 200
    offset_far_right = 1000
    near_cond_left = left_fitx_act[-1] > (left.bestx[-1] + offset_near_left) or left_fitx_act[-1] < (left.bestx[-1] - offset_near_left)
    far_cond_left = left_fitx_act[0] > (left.bestx[0] + offset_far_left) or left_fitx_act[0] < (left.bestx[0] - offset_far_left)
    near_cond_right = right_fitx_act[-1] > (right.bestx[-1] + offset_near_right) or right_fitx_act[-1] < (right.bestx[-1] - offset_near_right)
    far_cond_right = right_fitx_act[0] > (right.bestx[0] + offset_far_right) or right_fitx_act[0] < (right.bestx[0] - offset_far_right)
    
    if near_cond_left or far_cond_left: 
        print(left.bestx.shape)
        left_fitx = np.copy(left.bestx.reshape(1440))
        left.detected = False
    else:
        left.detected = True
    if near_cond_right or far_cond_right:
        right_fitx = np.copy(right.bestx.reshape(1440))
        right.detected = False
    else:
        right.detected = True
              
    # if a lane is detected, add the fit curve to the allx struct
    # then average the last n curves in allx to store as bestx
    n = 3  # number of previous frames to average best x 
  
    if left.detected == True:
        print(left_fitx.shape)
        left.allx = np.append(left.allx,left_fitx.reshape(1440,1), axis=1)
    if left.allx.shape[1] < n:
        left.bestx = np.mean(left.allx, axis = 1)
    else:
        left.bestx = np.mean(left.allx[:,-n:], axis = 1)

    if right.detected == True:
        right.allx = np.append(right.allx,right_fitx.reshape(1440,1), axis=1)
    if right.allx.shape[1] < n:
        right.bestx = np.mean(right.allx, axis = 1)
    else:
        right.bestx = np.mean(right.allx[:,-n:], axis = 1)

    #polynomial coefficients for the most recent fit
    left.current_fit = left_fit_cr
    right.current_fit = right_fit_cr  
    
    #radius of curvature and vehicle center
    lane_width, lane_deviation_from_center = measure_center(final_orig, left_fitx, right_fitx)
    K_left, K_right = measure_curvature_real(ploty, left_fit_cr, right_fit_cr)
    left.radius_of_curvature = K_left
    right.radius_of_curvature = K_right
        
    result = polyfill(final_orig, test_img_orig, M, left_fitx, right_fitx, ploty)

    # add text to plot
    printToPic(result, K_left, K_right, lane_deviation_from_center)
    
    # for debugging
    #frame = frame + 1

    global top_right
    global bottom_right
    global bottom_left
    top = np.concatenate((result, top_right), axis=0)
    bottom = np.concatenate((bottom_left, out_img), axis=0)
    result = np.concatenate((top, bottom), axis=1)

    img = bridge.cv2_to_imgmsg(result,'bgr8')
    pub_image_quad.publish(img)
    
    return result

def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # Create an output image to draw on and visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETERS
    # Choose the number of sliding windows
    nwindows = 15
    # Set the width of the windows +/- margin
    margin = 350
    # Set minimum number of pixels found to recenter window
    minpix = 10

    # Set height of windows - based on nwindows above and image shape
    window_height = np.int(binary_warped.shape[0]//nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        ### TO-DO: Find the four below boundaries of the window ###
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin/3
        win_xright_high = rightx_current + margin/3
        
        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_xleft_low,win_y_low*3),
        (win_xleft_high,win_y_high*3),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xright_low,win_y_low),
        (win_xright_high,win_y_high),(0,255,0), 2) 
        
        ### TO-DO: Identify the nonzero pixels in x and y within the window ###
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        ### TO-DO: If you found > minpix pixels, recenter next window ###
        ### (`right` or `leftx_current`) on their mean position ###
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty, out_img



def polyfill(top_image, img_orig, M, left_fitx, right_fitx, ploty):

    # Create an image to draw the lines on
    warp_zero = np.zeros_like(top_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    Minv = np.linalg.inv(M)
    newwarp = cv2.warpPerspective(color_warp, Minv, (img_orig.shape[1], img_orig.shape[0])) 
    # Combine the result with the original image
    result = cv2.addWeighted(img_orig, 1, newwarp, 0.3, 0)
    return result




def fit_polynomial(binary_warped):
    # Find our lane pixels first
    leftx, lefty, rightx, righty, out_img = find_lane_pixels(binary_warped)

    ### TO-DO: Fit a second order polynomial to each using `np.polyfit` ###
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    #print ploty.shape
    try:
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    except TypeError:
        # Avoids an error if `left` and `right_fit` are still none or incorrect
        print('The function failed to fit a line!')
        left_fitx = 1*ploty**2 + 1*ploty
        right_fitx = 1*ploty**2 + 1*ploty

    ## Visualization ##
    # Colors in the left and right lane regions
    out_img[lefty, leftx] = [255, 0, 0]
    out_img[righty, rightx] = [0, 0, 255]

    # Plots the left and right polynomials on the lane lines
    #plt.plot(left_fitx, ploty, color='yellow')
    #plt.plot(right_fitx, ploty, color='yellow')

    return out_img, ploty, left_fit, right_fit, left_fitx, right_fitx

def measure_curvature_real(ploty, left_fit_cr, right_fit_cr):
    '''
    Calculates the curvature of polynomial functions in meters.
    '''
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension
    
    # Define y-value where we want radius of curvature
    # We'll choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)
    
    # Implement the calculation of R_curve (radius of curvature) #####
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

    
    return left_curverad, right_curverad



def process_image_test(image):
    test_img = image
    test_img_orig = test_img.copy()
    offset = 400
    offset2 = 400
    offset3 = 400
    cv2.line(test_img_orig, (190+offset,719+offset3), (591+offset, 448+offset3), [255,0,0], 5)
    cv2.line(test_img_orig, (687+offset2,448+offset3), (1126+offset2, 719+offset3), [255,0,0], 5)         
    test_img, M = unwarp_lane2(test_img,mtx,dist)
    combined, final = pipeline(test_img)
    final = cv2.cvtColor(final,cv2.COLOR_BGR2GRAY)
    final_orig = final.copy()
    out_img, ploty, left_fit_cr, right_fit_cr, left_fitx, right_fitx = fit_polynomial(final)
    result = polyfill(final_orig, test_img_orig, M, left_fitx, right_fitx, ploty)
    
    return test_img_orig



def measure_center(img,left_fitx,right_fitx):
    lane_width_p = right_fitx[-1] - left_fitx[-1]
    xm_per_pix = 3.7 / lane_width_p
    
    x_pixel_dim = img.shape[0]
    x_pixel_center = x_pixel_dim/2.00
    deviation_from_center = (x_pixel_center - (left_fitx[-1] + lane_width_p/2))*xm_per_pix 

    return lane_width_p*xm_per_pix, deviation_from_center


def pub_scatter(left, right):
    
    try:
        if right.bestx is not None and left.bestx is not None:
            # calculate lane coordinates in real space
            pix_per_m = 485 # pixels/ft
            bottom_to_center = 2 # m

            tupled_pix_left = zip(left.bestx,left.ploty,np.zeros(left.bestx.shape))
            tupled_pix_right = zip(right.bestx,right.ploty,np.zeros(right.bestx.shape))
            
            transform = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
            
            

            shift =0 # m
            cloud_pix_left_lane = []
            for tup in tupled_pix_left:
                temp = np.matmul(tup,transform) + np.array([1440,960,0])
                temp[1] = 0.005061*temp[1] + shift
                temp[0] = 0.0115062345*temp[0] # convert pixels to actial distance
                point_msg = Point32() 
                point_msg.x = temp[0]
                point_msg.y = temp[1]
                point_msg.z = temp[2]
                cloud_pix_left_lane.append(point_msg)
            

            cloud_pix_right_lane = []
            for tup in tupled_pix_right:
                temp = np.matmul(tup,transform) + np.array([1440,960,0])
                
                temp[1] = 0.005061*temp[1] + shift
                temp[0] = 0.0115062345*temp[0] # convert pixels to actial distance
                point_msg = Point32() 
                point_msg.x = temp[0]
                point_msg.y = temp[1]
                point_msg.z = temp[2]
                #print point_msg
                cloud_pix_right_lane.append(point_msg)

            #print type(cloud_pix_right_lane)

                



            return cloud_pix_left_lane, cloud_pix_right_lane

    except ValueError:
        pass
        return None



def callback(data):

	global left
	global right
	#global frame
	img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
	image = process_image(img)
	image = bridge.cv2_to_imgmsg(image,'bgr8')
	print("lol")
	pub_image.publish(image)
	left_points, right_points = pub_scatter(left, right)

	left_msg=PointCloud()
	left_msg.header.stamp = rospy.Time.now()
	left_msg.header.frame_id = "footprint"
	left_msg.points=left_points
	left_msg_chan=ChannelFloat32()
	left_msg_chan.name = 'value'
	left_msg_chan.values = [float(3)]
	left_msg.channels=[left_msg_chan]

	right_msg=PointCloud()
	right_msg.header.stamp = rospy.Time.now()
	right_msg.header.frame_id = "footprint"
	right_msg.points=right_points
	right_msg_chan=ChannelFloat32()
	right_msg_chan.name = 'value'
	right_msg_chan.values = [float(3)]
	right_msg.channels=[right_msg_chan]

	pub_point_cloud_right.publish(right_msg)
	pub_point_cloud_left.publish(left_msg)



def lane_detection_udacity():
	rospy.init_node('lanedetection',anonymous=True)
	rospy.Subscriber("/camera_fm/camera_fm/image_raw",Image,callback,queue_size=1,buff_size=2**24)
	#rospy.Subscriber("/camera/image_color",Image,callback,queue_size=1,buff_size=2**24)
	try:
		rospy.loginfo("Entering ROS Spin")
		#rospy.spin()
		rate = rospy.Rate(12) # hz
		while not rospy.is_shutdown():
			rate.sleep()

	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	try:
		lane_detection_udacity()
	except rospy.ROSInterruptException:
		pass
