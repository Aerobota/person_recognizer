#!/usr/bin/env python
import roslib
import caffe
import cv2
import numpy as np
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

def normalize(arr):
    """
    Linear normalization
    http://en.wikipedia.org/wiki/Normalization_%28image_processing%29
    """
    # Do not touch the alpha channel
    for i in range(3):
        minval = arr[...,i].min()
        maxval = arr[...,i].max()
        if minval != maxval:
            arr[...,i] -= minval
            arr[...,i] *= (255.0/(maxval-minval))
    return arr

class GenderRecognizer:
    def __init__(self):
        os.chdir("/home/"+os.getenv('USER')+"/catkin_ws/src/person_recognizer")

        self.net = caffe.Classifier(
            "cfg/deploy.prototxt",
            "cfg/snapshot_iter_510.caffemodel"
        )
        self.labels = {0:"man",1:"woman"}
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/faces",Image,self.callback)
        self.gender_pub = rospy.Publisher("/gender",String)

    def callback(self,data):
        try:
            image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError, e:
            print e
            
        image = normalize(image.astype(np.float32))    
        score = self.net.predict([image], oversample = False)
        self.gender_pub.publish(self.labels[np.argmax(score)])
            
def main(args):
    gender_recognizer = GenderRecognizer()
    rospy.init_node("gender_recognizer",anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shutting down")    

if __name__=="__main__":
    main(sys.argv)

