#!/usr/bin/env python3

'''
A ROS node that listens to user input all the time, publishes messages upon arrival of a certain input.
Then the job manager will be in charge of state transition.
'''
from speech_recognition import speechRecog
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
import rospy

states_map = {'cancel': 0,
              'left': 2, 'middle': 3, 'right': 4, 'nothing': 5}

keywords_map = {'left': 2, 'middle': 3, 'right': 4, 'nothing': 5, 
                'cancel': 8, 'terminate': 9, 'end': 10, 'stop': 11}

class AudioListener:
    def __init__(self):
        self.counter = 0
        self.user_cmd_pub = rospy.Publisher("/suitbot/audio/cmd_in", Int32, queue_size=1)
        self.set_listening_service = rospy.Service("/suitbot/audio/set_listening", SetBool, self.callback_set_listening)
        self.r = rospy.Rate(4)
        self.listening = False
        
    def callback_set_listening(self, req):
        if req.data == True:
            print("Audio Listener: Start listening..")
        else: 
            print("Audio Listener: Stop listening..")
        self.listening = req.data
        return (True, None)

    def loop(self):
        global keywords_map, states_map
        print("Audio Listener entering loop")
        while not rospy.is_shutdown():
            if self.listening:
                keyword_idx = speechRecog(keywords_map)
                msg_out = Int32()
                if keyword_idx == keywords_map['cancel'] or keyword_idx == keywords_map['terminate'] or \
                        keyword_idx == keywords_map['end'] or keyword_idx == keywords_map['stop']:
                    msg_out.data = states_map['cancel']
                else:
                    msg_out.data = keyword_idx
                self.user_cmd_pub.publish(msg_out)
            self.r.sleep()
        


if __name__ == '__main__':
    print("Audio Listener node starting")
    rospy.init_node('audio_listener')
    listener = AudioListener()
    listener.loop()
    print("Audio Listener going into spin...")
    rospy.spin()