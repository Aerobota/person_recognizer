#!/usr/bin/env python

#system modules
import sys
import os
#ros system modules
import rospy
import roslib
#numpy module
import numpy as np
#image processing and machine learning module
import cv2
import caffe
#std_msgs module
from std_msgs.msg import String, Header
#sensor_msgs module
from sensor_msgs.msg import Image
#person_recognizer messages modules
from person_recognizer.msg import Face, Gender, SimpleImage
#cv_bridge modules
from cv_bridge import CvBridge, CvBridgeError

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
        self.image_sub = rospy.Subscriber("/faces",Face,self.callback)
        self.gender_pub = rospy.Publisher("/gender",Gender)

    def callback(self,data):
        msg = Gender()

        faces = [SimpleImage() for i in range(data.number)]
        
        for i in range(data.number):
            
            #copy Face topic image buffers
            faces[i].width = data.faces[i].width
            faces[i].height = data.faces[i].height
            faces[i].data = data.faces[i].data

            #convert sensor_msgs/Image to cv::Mat
            try:
                image = self.bridge.imgmsg_to_cv2(data.faces[i],"bgr8")
            except CvBridgeError, e:
                print e
                
            #normalize image and predict gender
            image = normalize(image.astype(np.float32))    
            score = self.net.predict([image], oversample = False)
            #queue gender
            msg.gender.append(self.labels[np.argmax(score)])

        msg.faces = faces
        msg.header.stamp = rospy.Time.now()
        self.gender_pub.publish(msg)
            

def main(args):
    rospy.init_node("gender_recognizer",anonymous=True)
    gender_recognizer = GenderRecognizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shutting down")    

if __name__=="__main__":
    main(sys.argv)

