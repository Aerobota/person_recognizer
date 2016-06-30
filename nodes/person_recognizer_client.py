#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image, Imu
from person_recognizer.msg import Gender, Face, Person
from geometry_msgs.msg import Twist, Vector3
from futaba_driver.msg import Servo
from object_recognizer.msg import Person 
from subprocess import call
import numpy as np
import time
import os
import math
import thread

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
    speed = 0
    current_angle = 0
    serif = 0
    num_people = 0

    @staticmethod
    def ImuCb(data):
        if abs(data.angular_velocity.z) > 0.01:
            PersonRecognizerClient.speed = data.angular_velocity.z
        else:
             PersonRecognizerClient.speed = 0.0

    @staticmethod
    def locCb(data):
       PersonRecognizerClient.num_people = len(data.centroids)

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
            self.speak("Operator, tell your name")
            self.speak("by this sentence only.")
            self.speak("I am Bob, I am John")
            self.speak("2nd")
            self.speak("Operator, go to the front of Mini Robot.")
            self.speak("Make your face straight")
            self.speak("3rd")
            self.speak("Very important thing.")
            self.speak("Operator, say it loud as you can.")
            self.speak("Operator, say it loud as you can.")
            self.speak("Operator, say it loud as you can.")
            self.speak("OK?")
            PersonRecognizerClient.state = 1

        elif PersonRecognizerClient.state == 1:
            self.speak("I am going to remember you.")
            self.speak("Say your name")
            self.speak("3")
            self.speak("2")
            self.speak("1")
            self.speak("go")
            speech_control_pub.publish("speak")
            time.sleep(3)
            PersonRecognizerClient.state = 2

        elif PersonRecognizerClient.state == 2:
            if PersonRecognizerClient.name is "":
                self.speak("Waiting for operator talking")
            else:
                rospy.loginfo("serif : %d", PersonRecognizerClient.serif)
                if PersonRecognizerClient.serif is 1 or PersonRecognizerClient.serif is 0:
                    self.speak("Make your face straight")
                elif PersonRecognizerClient.serif== 2:
                    self.speak("going near you")
                    vel = Twist()
                    vel.linear.x = 1.0
                    vel_pub.publish(vel)
                    time.sleep(0.5)
                elif PersonRecognizerClient.serif == 3:
                    self.speak("turning to find your face")
                    vel = Twist()
                    vel.angular.z = -1.0
                    vel_pub.publish(vel)
                    time.sleep(0.5)

                elif PersonRecognizerClient.serif % 2 == 0:
                    self.speak("go near me")
                    vel = Twist()
                    vel.angular.z = 1.0
                    vel_pub.publish(vel)
                    time.sleep(0.5)
                    vel_pub.publish(vel)

                elif PersonRecognizerClient.serif % 2 == 1:
                    self.speak("turning to find your face")
                    vel = Twist()
                    vel.angular.z = -1.0
                    vel_pub.publish(vel)
                    time.sleep(0.5)
                    vel_pub.publish(vel)

                else:
                    self.speak("your name is " + PersonRecognizerClient.name)
                print "faces number : ", len(data.faces)
                if len(data.faces) > 0:
                    self.remembered_face = data.faces[np.argmin(map(lambda x : x,data.xangle))]
                    person_recog_pub.publish(self.remembered_face)
                    speech_control_pub.publish("stop")
                    person_state_pub.publish(1)
                    #time.sleep(1)
                    PersonRecognizerClient.state = 3 
                else:
                    PersonRecognizerClient.serif += 1
                    
        elif PersonRecognizerClient.state == 3:
            self.speak("waiting 10 seconds")
            time.sleep(10)
            PersonRecognizerClient.state = 4
            self.speak("finding operator")
            self.speak("first turning")

        elif PersonRecognizerClient.state == 4:
                vel = Twist()
                vel.angular.z = 3.14
                vel_pub.publish(vel)
                time.sleep(1.0)
                vel_pub.publish(vel)
                time.sleep(1.0)
                vel.angular.z = 1.4
                vel_pub.publish(vel)
                time.sleep(1.0)
                vel = Twist()
                vel.linear.x = 1.0
                vel_pub.publish(vel)
                time.sleep(1.0)
                
                vel_pub.publish(vel)
                PersonRecognizerClient.state = 5
        
        elif PersonRecognizerClient.state == 5:
            self.speak("finding operator")
            self.speak("Everyone")
            self.speak("Make your face straight")
            self.speak("Make your face straight")
            self.speak("Make your face straight")

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

                    self.speak("operator "+ PersonRecognizerClient.name + " relative position from Happy Mini")
                        
                    self.speak("at azimuth angle " + str(PersonRecognizerClient.person_info.xangle[self.person_index]))
                    twist_angle = -PersonRecognizerClient.person_info.xangle[self.person_index]
                    self.speak("at elevation angle " + str(-PersonRecognizerClient.person_info.yangle[self.person_index]/2))
                    self.speak("operator " + PersonRecognizerClient.name + " gender is " + PersonRecognizerClient.person_info.gender[self.person_index])

                    self.speak("I am going to point you with my arm")
                    self.speak("and my direction")

                    vel = Twist()
                    vel.angular.z = math.radians(twist_angle)
                    vel_pub.publish(vel)

                    servo = Servo()
                    servo.time = [3 for i in range(6)]
                    servo.idx    = [i+1 for i in range(6)]
                    servo.torque = [1, 1, 1, 1, 1, 1]
                    servo.angle = [-PersonRecognizerClient.person_info.yangle[self.person_index]/2 + 140, -PersonRecognizerClient.person_info.yangle[self.person_index]/2 + 140, -140, 140, 15, 0]
                    arm_pub.publish(servo)
                    time.sleep(1.0)
                    arm_pub.publish(servo)
                    time.sleep(1.0)
                    arm_pub.publish(servo)
                    time.sleep(1.0)
                    rospy.loginfo("moving arm")

                    self.speak("Happy mini pointed you, " + PersonRecognizerClient.name)
                    time.sleep(1.0)

            else:
                self.speak("Waiting for Finding Operator" + PersonRecognizerClient.name)

        elif PersonRecognizerClient.state == 7:

            if person_info_received is True:
                women_num = 0
                men_num = 0
                sitting_num = 0
                standing_num = 0

                for i in range(len(PersonRecognizerClient.person_info.faces)):
                    if PersonRecognizerClient.person_info.gender[i] == "man":
                        men_num += 1
                    else:
                        women_num += 1
                    if PersonRecognizerClient.person_info.yangle[i] > 0:
                        sitting_num += 1
                    else:
                        standing_num += 1

                if len(PersonRecognizerClient.person_info.faces) > 0:
                    self.speak("the number of human is ")
                    self.speak(str(PersonRecognizerClient.num_people))
                    self.speak("number of man is " +str(men_num))
                    self.speak("number of woman is " + str(women_num))
                    self.speak("number of people undentified is " + str(PersonRecognizerClient.num_people - men_num - women_num))
                    self.speak("number of people sitting is " + str(sitting_num))
                    self.speak("number of people standing is " +str(standing_num))
                    PersonRecognizerClient.state = 8

                else:
                    self.speak("finding people")
                    vel = Twist()
                    vel.linear.z = 0.5
                    vel_pub.publish(vel)
                    time.sleep(1.0)

            else:
                rospy.loginfo("waiting for recognition result")

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
    imu_sub = rospy.Subscriber("/mobile_base/sensors/imu_data_raw", Imu, person_rec_cli.ImuCb)
    num_sub = rospy.Subscriber("/human_location", Person, person_rec_cli.locCb)
    rospy.spin()
