#!/usr/bin/env python
import sys
import rospy
import math
import nltk
from compiler.ast import flatten
from nltk import *
from nltk.corpus import wordnet as wn
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class GPSR:
    def __init__(self):
        self.speech = rospy.Subscriber("/voice_recog", String,self.speechcallback)
        self.speech_input = ""
        self.token = []
        self.token_tag = []
        self.soundhandle = SoundClient()
        self.voice = "voice_kal_diphone"
        self.name_pub = rospy.Publisher("/name", String, queue_size=10)
        
    def tokenize(self,s_input):
        self.token = word_tokenize(s_input)
        if len(self.token) > 0:
            self.token[0] = self.token[0].lower()

        self.token_tag = pos_tag(self.token)

        token_tag_new = [list(self.token_tag[i]) for i in range(len(self.token_tag))]
        for i in range(len(self.token_tag)):
            if self.token[i] == "move" or self.token[i] == "grasp":
                token_tag_new[i][1] = "VB"
            if self.token[i] == "bed" or self.token[i] == "couch" or self.token[i] == "noodles":
                token_tag_new[i][1] = "NN"
        self.token_tag = [tuple(token_tag_new[i]) for i in range(len(self.token_tag))]
        return self.token

    def synonym(self,s_input):
        synset_list = wn.synsets(s_input)
        lemmas = flatten([synset_list[i].lemma_names() for i in range(len(synset_list))])
        vocab = list(set(lemmas))
        return vocab

    def PRPProcessor(self):
        predict = []
        for i in range(len(self.token_tag)):
            if self.token_tag[i][1] == "NN" or self.token_tag[i][1] == "NNP":
                predict.append(i)
        continuous = []
        for i in range(len(self.token_tag)):
            mini = [10000,len(predict)-1]
            if self.token_tag[i][1] == "PRP":
                if self.token_tag[i][0] == "yourself":
                    self.token[i] = "Mini"
                    self.token_tag[i] = tuple(["Mini","NNP"])
                elif self.token_tag[i][0] == "me":
                    self.token[i] = "operator"
                    self.token_tag[i] = tuple(["operator","NNP"])
                else:
                    for j in range(len(predict)):
                        if mini[0] > i - predict[j] and i - predict[j] > 0:
                            mini = [i-predict[j],predict[j]]
                        j = mini[1]
                        while j > 0:
                            if self.token_tag[j-1][1] == "NNP" or self.token_tag[j-1][1] == "NN":
                                continuous.append(self.token[j-1])
                            else: break
                            j-= 1
                        continuous.reverse()

                        if len(continuous) > 0:
                            self.token[i] = " ".join(continuous)+" "+self.token[mini[1]]
                        else:
                            self.token[i] = self.token[mini[1]]
                            print self.token[i]
                            self.token_tag[i] = tuple([self.token[i], "NNP"])

    def speechcallback(self,data):
        #print data
        token = self.tokenize(str(data).replace("data:",""))
        self.PRPProcessor()
        syn_list = []
        
        print "synonyms"
        _syn = []
        for i in range(len(syn_list)):
            syn = pos_tag(syn_list[i])
            __syn = []
            for j in range(len(syn)):
                if syn[j][1] == self.token_tag[i][1]:
                    if self.token_tag[i][1] != "NNP":
                        __syn.append(syn[j][0])
            syn.append(__syn)

        msg = String()
        msg.data = token[len(token)-1]
        print "name : ", msg.data
        self.name_pub.publish(msg)

def main(args):
    rospy.init_node("name_extractor")
    gpsr = GPSR()
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
