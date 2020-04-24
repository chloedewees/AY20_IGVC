"""Most of this code was taken from the darknet github, but we made certain changes
to make this useful as a Ros node"""



#!/usr/bin/env python
from ctypes import *
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray
from AY20_IGVC.msg import VisualObject, VisualObjectArray
import cv2
import math
import random
from cv_bridge import CvBridge, CvBridgeError

net = None
meta = None
obj_array = VisualObjectArray()
pub_state = UInt16MultiArray()
pub_state.data = []
pub = rospy.Publisher('detection_status',UInt16MultiArray,queue_size=1)
objects_pub = rospy.Publisher('/visual_objects', VisualObjectArray, queue_size=1)

def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1
    
bridge = CvBridge()

def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr

class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]

class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]

class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]

    

#lib = CDLL("/home/pjreddie/documents/darknet/libdarknet.so", RTLD_GLOBAL)
lib = CDLL("libdarknet.so", RTLD_GLOBAL)
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

set_gpu = lib.cuda_set_device
set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)

def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        res.append((meta.names[i], out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res

def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
    im = load_image(image, 0, 0)
    num = c_int(0)
    pnum = pointer(num)
    predict_image(net, im)
    dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, None, 0, pnum)
    num = pnum[0]
    if (nms): do_nms_obj(dets, num, meta.classes, nms);
    ### The below loop collects yoloV3's detections and formats it for publication
    res = []
    for j in range(num):
        for i in range(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                res.append((meta.names[i], dets[j].prob[i], (b.x, b.y, b.w, b.h)))
                if meta.names[i] == "stop sign":
                    pub_state.data.append(b.w * b.h)
                    obj_array.append([0, b.x, b.y, b.w * b.h])
    res = sorted(res, key=lambda x: -x[1])
    free_image(im)
    free_detections(dets, num)
    return res

### This our callback, it formats computer vision images to a .jpg format 
def callback(data):
    global pub_state
    try:
        cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        cv2.imwrite('data/camera_image.jpg', cv2_img)
        
    r = detect(net, meta, "data/camera_image.jpg")
    ##print r
    
def listener():
    global pub_state
    rospy.init_node('sign_detection', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    topic = "camera_fm/camera_fm/image_raw"
    rospy.Subscriber(topic, Image, callback)
    while not rospy.is_shutdown():
        print(pub_state)
        pub.publish(pub_state)
        objects_pub.publish( )
        rate.sleep()
        
if __name__ == "__main__":
    ## The pathing info here is important if you guys change your file organiztion change this stuff and the
    ## paths in the coco data file
    net = load_net("/home/user1/catkin_ws/src/get_hsv/darknet_test/cfg/yolov3.cfg", "/home/user1/catkin_ws/src/get_hsv/darknet_test/yolov3.weights", 0)
    meta = load_meta("/home/user1/catkin_ws/src/get_hsv/darknet_test/cfg/coco.data")
    #r = detect(net, meta, "data/dog.jpg")
    #print r
    try:
        listener()
    except KeyboardInterrupt:
        print("Goodbye")
