#!/usr/bin/env python3
from gtts import gTTS

import sounddevice as sd
import numpy as np

import os
from os import path
from pydub import AudioSegment

import soundfile as sf
import rospy
from suitbot_ros.srv import SpeechSrv
import random

MAX_NUM = 10000000

class S:
    def __init__(self, record_time):
        self.totalSound = 0
        self.totalCount = 0
        self.record_time = record_time

    def print_sound(self, indata, outdata, frames, time, status):
        volume_norm = np.linalg.norm(indata)*5
        self.totalCount += 1
        self.totalSound += volume_norm
        print ("|" * int(volume_norm))

    def okay(self):
        with sd.Stream(callback=self.print_sound):
            sd.sleep(self.record_time*1000) # ms




class AudioSpeaker:
    def __init__(self):
        self.b = S(2)
        self.b.okay()
        self.text_sub = rospy.Service("/suitbot/audio/speech_output", SpeechSrv, self.callback_text)

    def getSound(self, wavFile):
        print(self.b.totalSound, self.b.totalCount)
        noise = self.b.totalSound / self.b.totalCount
        print("environment noise:", self.b.totalSound / self.b.totalCount)

        weight = noise / 10
        data, fs = sf.read(wavFile)
        sd.play(weight*data, fs)
        sd.wait()

    def stringtoSpeech(self, mytext):
        # Language in which you want to convert
        language = 'en'
        # Passing the text and language to the engine, here we have marked slow=False. Which tells
        # the module that the converted audio should have a high speed
        myobj = gTTS(text=mytext, lang=language, slow=False)
        # Saving the converted audio in a mp3 file named welcome
        rn_str = str(random.randint(1, MAX_NUM))
        src = rn_str + ".mp3"
        myobj.save(src)
        # Playing the converted file
        #os.system("welcome.mp3")
        dst = rn_str + ".wav"
        # convert mp3 to wav
        sound = AudioSegment.from_mp3(src)
        sound.export(dst, format="wav")
        self.getSound(dst)
        os.remove(src)
        os.remove(dst)
        
    def callback_text(self, req):
        text = req.data
        self.stringtoSpeech(text)
        return True
        


if __name__ == '__main__':
    print("Audio Speaker node starting")
    rospy.init_node('audio_speaker')
    speaker = AudioSpeaker()
    print("Audio Speaker going into spin...")
    rospy.spin()

#stringtoSpeech("Tina likes functions")