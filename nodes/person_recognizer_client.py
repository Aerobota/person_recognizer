#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
from person_recognizer.msg import Gender, Face, Person
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import time


speech_control_pub = rospy.Publisher("/speech_control", String)
person_recog_pub = rospy.Publisher("/person_recog_faces", Image)
person_state_pub = rospy.Publisher("/record_faces", Int32)
vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        
person_info_received = False


class PersonRecognizerClient:
    person_info = Person()
    name = ""
    state = 0


    @staticmethod
    def setName(data):
        PersonRecognizerClient.name = data.data


    @staticmethod
    def setPersonInfo(data):
        rospy.loginfo("receive person info")
        PersonRecognizerClient.person_info = data
        global person_info_received
        person_info_received = True
        

    def __init__(self):
        #face subscriber
        self.face_sub = rospy.Subscriber("/faces", Face, self.EventLoop)
        #velocity publisher
        self.remembered_face = Image()

    @classmethod
    def setLikelihood(self, data):
        self.likelihood = data

    @classmethod
    def EventLoop(self, data):
        rospy.loginfo("state : %d", PersonRecognizerClient.state)
        if PersonRecognizerClient.state == 0:
            print "Operator, I am going to tell you 2 things." 
            print "1st"
            print "Operator, tell your name by this sentence only."
            print "I am Bob, I am John"
            print "2nd"
            print "Operator, go to the front of Mini Robot."
            print "OK?"
            PersonRecognizerClient.state = 1

        elif PersonRecognizerClient.state == 1:
            print "I am going to remember you."
            print "Say your name"
            speech_control_pub.publish("speak")
            PersonRecognizerClient.state = 2

        elif PersonRecognizerClient.state == 2:
            time.sleep(1)
            print "your name is ", PersonRecognizerClient.name
            self.remembered_face = data.faces[np.argmin(map(lambda x : x,data.xangle))]
            person_recog_pub.publish(self.remembered_face)
            speech_control_pub.publish("stop")
            person_state_pub.publish(1)
            time.sleep(1)
            PersonRecognizerClient.state = 3 

        elif PersonRecognizerClient.state == 3:
            print "waiting 10 seconds"
            #time.sleep(10)
            PersonRecognizerClient.state = 4

        elif PersonRecognizerClient.state == 4:
            print "finding operator"
            print "first turning", PersonRecognizerClient.name
            vel = Twist()
            vel.angular.z = 3.14
            #vel_pub.publish(vel)
            time.sleep(1)
            PersonRecognizerClient.state = 5
        
        elif PersonRecognizerClient.state == 5:
            print "finding operator"
            print "scanning faces"
            person_state_pub.publish(2)
            time.sleep(1)
            PersonRecognizerClient.state = 6

        elif PersonRecognizerClient.state == 6:
            if len(PersonRecognizerClient.person_info.faces) > 0:
                person_index  = np.argmin(PersonRecognizerClient.person_info.distance)
                print "face location", person_index
                print "xangle : ", PersonRecognizerClient.person_info.xangle
                print "yangle : ", PersonRecognizerClient.person_info.yangle
                print "operator" , PersonRecognizerClient.name, "is at azimuth", PersonRecognizerClient.person_info.xangle[person_index], "elevation", PersonRecognizerClient.person_info.yangle[person_index]
                print "operator", PersonRecognizerClient.name, "gender is", PersonRecognizerClient.person_info.gender[person_index]
            else:
                print "operator", PersonRecognizerClient.name , "not found"
            PersonRecognizerClient.state = 7

        elif PersonRecognizerClient.state == 7:
            print "face number", len(PersonRecognizerClient.person_info.faces)
            if person_info_received is True:
                for i in range(len(PersonRecognizerClient.person_info.faces)):
                    print "person at azimuth", PersonRecognizerClient.person_info.xangle[person_index], "elevation", PersonRecognizerClient.person_info.yangle[person_index], "gender is", PersonRecognizerClient.person_info.gender[person_index]
                PersonRecognizerClient.state = 8
            else:
                rospy.loginfo("waiting for person info receive")
        elif PersonRecognizerClient.state == 8:
            print "making pdf report"
            time.sleep(1)
            print "ending"
            PersonRecognizerClient.state = 9

        

if __name__ == "__main__":
    rospy.init_node("person_recognizer_client")
    person_rec_cli = PersonRecognizerClient()
    person_sub = rospy.Subscriber("/person", Person, person_rec_cli.setPersonInfo)
    name_sub = rospy.Subscriber("/name",String, person_rec_cli.setName)

    rospy.spin()