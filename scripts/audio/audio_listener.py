#!/usr/bin/env python3

'''
A ROS node that listens to user input all the time, publishes messages upon arrival of a certain input.
Then the job manager will be in charge of state transition.
'''
from speech_recognition import *
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
import rospy

import sys, os.path
script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(script_dir)
from parameters import Parameters


cancel_words = ['cancel', 'terminate', 'end', 'stop']

class AudioListener:
    def __init__(self, model_path):
        self.counter = 0
        self.speech_recognizer = SpeechRecognizer(model_path)
        self.user_cmd_pub = rospy.Publisher(parameters.usr_cmd_topic, Int32, queue_size=1)
        self.set_listening_service = rospy.Service(parameters.listening_service, SetBool, self.callback_set_listening)
        self.r = rospy.Rate(4)
        self.listening = False
        
    def callback_set_listening(self, req):
        if req.data == True:
            rospy.loginfo("Audio Listener: lisening enabled")
        else: 
            rospy.loginfo("Audio Listener: lisening disabled")
        self.listening = req.data
        return (True, None)

    def loop(self):
        global cancel_words
        keywords_map = parameters.keywords_map
        states_map = parameters.states_map
        rospy.loginfo("Audio Listener: entering loop")
        while not rospy.is_shutdown():
            if self.listening:
                keyword_idx = self.speech_recognizer.speechRecog(keywords_map)
                msg_out = Int32()
                assert(keyword_idx < len(keywords_map))
                wd = keywords_map[keyword_idx]
                if wd in cancel_words:
                    msg_out.data = states_map['cancel']
                else:
                    msg_out.data = states_map[wd]
                self.user_cmd_pub.publish(msg_out)
            self.r.sleep()
        


if __name__ == '__main__':
    parameters = Parameters()
    parameters.initParameters()
    rospy.loginfo("Audio Listener: node starting")
    rospy.init_node('audio_listener')
    listener = AudioListener(parameters.model_path)
    listener.loop()
    rospy.loginfo("Audio Listener: node going into spin")
    rospy.spin()