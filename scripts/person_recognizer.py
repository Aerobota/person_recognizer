#!/usr/bin/env python
import sys
import os

import rospy
import cv2
import openface

from std_msgs.msg import Float32

from person_recognizer.msg import Gender, SimpleImage, Face

class PersonRecognizer:
    def __init__(self):
        os.chdir("/home/"+ os.getenv('USER') + "/catkin_ws/src/person_recognizer")
        self.align = openface.AlignDlib("models/shape_predictor_68_face_landmarks.dat")
        self.face_sub = rospy.Subscriber("/faces", Face, self.EventLoop)
        self.recog_sub = rospy.Subscriber("/record_faces", Int32, self.onRecogState)
        self.net = openface.TorchNeuralNet("models/nn4.small2.v1.t7",96)
        self.like_pub = rospy.Publisher("/face_likelihood", Float32, queue_size=10)
        self.recog_state = 0
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
                rospy.loginfo("Recorded Face.")
            else:
                rospy.loginfo("Face not found.")

        elif self.recog_state == 2:
            rospy.loginfo("State : Recognizing Faces")
            distance = Float32()
            distance.data = 1.0 - (self.getRep(self.data_record) -  self.getRep(data))
            rospy.loginfo("Face distance : %f", distance.data)
            self.like_pub.publish(distance)


    def getRep(self, data):
        rgb_buffer = map(lambda x : cv2.cvtColor(data.faces[x], cv2.COLOR_BGR2RGB), [0..len(data.faces)-1]) 

        bb = self.align.getLargestFaceBoundingBox(rgb_buffer)
        alignedFace = self.align.align(96, rgb_buffer, bb,
                                       landmarkIndices = openface.AlignDlib.OUTER_EYES_AND_NODE)

        distance = 1.0
        if alignedFace is None:
            rospy.loginfo("Unable to find face")
        else:
            distance = net.forward(alignedFace)

        return distance
            
            
