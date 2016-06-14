#!/usr/bin/env python
import sys
import os

import rospy
import cv2
import openface

from std_msgs.msg import Float32

from person_recognizer.msg import Gender, SimpleImage

class PersonRecognizer:
    def __init__(self):
        self.align = openface.AlignDlib("shape_predictor_68_face_landmarks.dat")
        self.net = openface.TorchNeuralNet("nn4.small2.v1.t7",96)
        self.like_pub = rospy.Publisher("/face_likelihood", )

    def getLikilihood(self, data):
        likelihood = Float32()
        likelihood.data = 1.0 - (getDistance(data_record) -  getDistance(data))
        self.like_pub = rospy.Publish(likelihood)

    def getDistance(self, data):
        bgr_buffer = []
        for i in range(height):
            bgr_buffer.append(data.data[(i-1):( (i * data.width) -1 )])
        bgr_buffer = np.asarray(bgr_buffer)

        rgb_buffer = cv2.cvtColor(bgr_buffer, cv2.COLOR_BGR2RGB)

        bb = self.align.getLargestFaceBoundingBox(rgb_buffer)
        alignedFace = self.align.align(96, rgb_buffer, bb,
                                       landmarkIndices = openface.AlignDlib.OUTER_EYES_AND_NODE)

        if alignedFace is None:
            rospy.loginfo("Unable to find face")
            return 1.0

        distance = net.forward(alignedFace)
            