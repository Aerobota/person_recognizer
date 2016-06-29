#!/usr/bin/env python
import sys
import os

import rospy
import cv2
import openface

from sklearn.pipeline import Pipeline
from sklearn.lda import LDA
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
from sklearn.mixture import GMM

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Int32
from person_recognizer.msg import Gender, Face, Person

class PersonRecognizer:
    def __init__(self):
        os.chdir("/home/"+ os.getenv('USER') + "/catkin_ws/src/person_recognizer")
        self.align = openface.AlignDlib("models/dlib/shape_predictor_68_face_landmarks.dat")
        self.face_sub = rospy.Subscriber("/gender", Gender, self.EventLoop)
        self.recog_sub = rospy.Subscriber("/record_faces", Int32, self.onRecogState)
        self.net = openface.TorchNeuralNet("models/openface/nn4.small2.v1.t7",96)
        self.person_pub = rospy.Publisher("/person", Person, queue_size=10)
        self.recog_state = 0
        self.faces = 0
        self.bridge = CvBridge()
        # state 0 : nothing, state 1 : record, state 3 : recognize

    def onRecogState(self, data):
        self.recog_state = data.data

    def EventLoop(self, data):
        if self.recog_state == 0:
            rospy.loginfo("State : Doing Nothing")

        elif self.recog_state == 1:
            rospy.loginfo("State : Recording Face")
            if len(data.faces) > 0:
                self.data_record = data.faces[0]
                print data.faces[0]
                rospy.loginfo("Recorded Face.")
            else:
                rospy.loginfo("Face not found.")

        elif self.recog_state == 2:
            rospy.loginfo("State : Recognizing Faces")
            person = Person()

            for i in range(len(data.faces)):
                d = 1.0 - (self.getRep(self.data_record) -  self.getRep(data.faces[i]))
                person.distance.append(np.dot(d,d))
                person.faces.append(data.faces[i])
                person.xangle.append(data.xangle[i])
                person.yangle.append(data.yangle[i])
                person.gender.append(data.gender[i])
                rospy.loginfo("Face distance : %f", person.distance[i])
        
            self.person_pub.publish(person)


    def getRep(self, data):
        rgb_buffer = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data, "bgr8"), cv2.COLOR_BGR2RGB) 

        bb = self.align.getLargestFaceBoundingBox(rgb_buffer)
        alignedFace = self.align.align(96, rgb_buffer, bb,
                                       landmarkIndices = openface.AlignDlib.OUTER_EYES_AND_NOSE)

        distance = 1.0
        if alignedFace is None:
            rospy.loginfo("Unable to find face")
        else:
            distance = self.net.forward(alignedFace)

        return distance

if __name__ == "__main__":
    rospy.init_node("person_recognizer")
    person_recog = PersonRecognizer()
    rospy.spin()
            
