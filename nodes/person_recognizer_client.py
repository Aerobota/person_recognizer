#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
from person_recognizer.msg import Gender, Face, Person
from geometry_msgs.msg import Twist, Vector3
from futaba_driver.msg import Servo
from subprocess import call
import numpy as np
import time
import os
import math

path = "/home/kendemu/catkin_ws/src/person_recognizer"

speech_control_pub = rospy.Publisher("/speech_control", String)
person_recog_pub = rospy.Publisher("/person_recog_faces", Image)
person_state_pub = rospy.Publisher("/record_faces", Int32)
arm_pub = rospy.Publisher("/servo", Servo)
vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        
person_info_received = False


class PersonRecognizerClient:
    person_info = Person()
    name = ""
    state = 0


    @staticmethod
    def speak(text):
        call(["./speak.sh", text, path])
        

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
        self.person_index = -1
        os.chdir(path)

    @classmethod
    def setLikelihood(self, data):
        self.likelihood = data

    @classmethod
    def EventLoop(self, data):
        rospy.loginfo("state : %d", PersonRecognizerClient.state)
        if PersonRecognizerClient.state == 0:
            self.speak("Operator, I am going to tell you 3 things.")
            self.speak("1st")
            self.speak("Operator, tell your name by this sentence only.")
            self.speak("I am Bob, I am John")
            self.speak("2nd")
            self.speak("Operator, go to the front of Mini Robot.")
            self.speak("3rd")
            self.speak("Operator, say it in a loud voice.")
            self.speak("OK?")
            PersonRecognizerClient.state = 1

        elif PersonRecognizerClient.state == 1:
            self.speak("I am going to remember you.")
            self.speak("Say your name")
            speech_control_pub.publish("speak")
            time.sleep(3)
            PersonRecognizerClient.state = 2

        elif PersonRecognizerClient.state == 2:
            time.sleep(1)
            if PersonRecognizerClient.name is "":
                self.speak("I am hearing you")
                self.speak("Please say it in a loud voice")
            else:
                self.speak("your name is " + PersonRecognizerClient.name)
                print "faces number : ", len(data.faces)
                if len(data.faces) > 0:
                    self.remembered_face = data.faces[np.argmin(map(lambda x : x,data.xangle))]
                    person_recog_pub.publish(self.remembered_face)
                    speech_control_pub.publish("stop")
                    person_state_pub.publish(1)
                    time.sleep(1)
                    PersonRecognizerClient.state = 3 
                else:
                    self.speak("I am finding you")
                    self.speak("Go front of minis")
                    self.speak("look at mini eye.")


        elif PersonRecognizerClient.state == 3:
            self.speak("waiting 10 seconds")
            time.sleep(10)
            PersonRecognizerClient.state = 4

        elif PersonRecognizerClient.state == 4:
            self.speak("finding operator")
            self.speak("first turning") 
            #PersonRecognizerClient.name
            vel = Twist()
            vel.angular.z = 3.14
            vel_pub.publish(vel)
            time.sleep(1.0)
            vel_pub.publish(vel)
            time.sleep(1.0)
            PersonRecognizerClient.state = 5
        
        elif PersonRecognizerClient.state == 5:
            self.speak("finding operator")
            self.speak("scanning faces")
            person_state_pub.publish(2)
            time.sleep(1)
            PersonRecognizerClient.state = 6

        elif PersonRecognizerClient.state == 6:
            if len(PersonRecognizerClient.person_info.faces) > 0:
                if person_info_received is True:
                    PersonRecognizerClient.state = 7
                    print "person_info_received", person_info_received
                    self.person_index  = np.argmin(PersonRecognizerClient.person_info.distance)
                    print "face location", self.person_index
                    print "xangle : ", PersonRecognizerClient.person_info.xangle
                    print "yangle : ", PersonRecognizerClient.person_info.yangle
                    self.speak("operator " + PersonRecognizerClient.name +" is at azimuth " + str(PersonRecognizerClient.person_info.xangle[self.person_index]) + " elevation " + str(PersonRecognizerClient.person_info.yangle[self.person_index]))
                    self.speak("operator " + PersonRecognizerClient.name + " gender is " + PersonRecognizerClient.person_info.gender[self.person_index])

                    self.speak("I am going to point you with my arm and my direction")
                    servo = Servo()
                    servo.idx    = [i+1 for i in range(6)]
                    servo.torque = [1, 1, 1, 1, 1, 0]
                    servo.angle = [-PersonRecognizerClient.person_info.yangle[self.person_index], -PersonRecognizerClient.person_info.yangle[self.person_index] , 90, 90, 0, 0]
                    print servo_angle[0], servo_angle[1]

                    vel = Twist()
                    vel.angular.z = math.radians(PersonRecognizerClient.person_info.xangle[self.person_index])
                    servo.time = [1 for i in range(6)]
                    servo.parent_frame = ["base_link" for i in range(6)]
                    arm_pub.publish(servo)
                    time.sleep(5.0)
                    vel_pub.publish(vel)
                    self.speak("moved arm")
                    time.sleep(1.0)

            else:
                self.speak("waiting for operator finding")

        elif PersonRecognizerClient.state == 7:
            self.speak("number of human is " + str(len(PersonRecognizerClient.person_info.faces)))
            if person_info_received is True:
                for i in range(len(PersonRecognizerClient.person_info.faces)):
                    self.speak("person at azimuth "+str(PersonRecognizerClient.person_info.xangle[i])+" elevation "+str(PersonRecognizerClient.person_info.yangle[i])+ " gender is " +  PersonRecognizerClient.person_info.gender[i])
                PersonRecognizerClient.state = 8
            else:
                rospy.loginfo("waiting for person info receive")
        elif PersonRecognizerClient.state == 8:
            self.speak("making pdf report")
            time.sleep(1)
            self.speak("ending")
            PersonRecognizerClient.state = 9

        

if __name__ == "__main__":
    rospy.init_node("person_recognizer_client")
    person_rec_cli = PersonRecognizerClient()
    person_sub = rospy.Subscriber("/person", Person, person_rec_cli.setPersonInfo)
    name_sub = rospy.Subscriber("/name",String, person_rec_cli.setName)

    rospy.spin()
