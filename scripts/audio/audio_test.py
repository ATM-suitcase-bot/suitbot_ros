#!/usr/bin/env python3
# Import the required module for text
# to speech conversion
from gtts import gTTS

# This module is imported so that we can
# play the converted audio
import os
import subprocess

from os import path
from pydub import AudioSegment
from pydub.playback import play
from playsound import playsound

import time

def stringtoSpeech(mytext):
# The text that you want to convert to audio
#mytext = 'Apple loves Tina and Mithril'

# Language in which you want to convert
    language = 'en'

    # Passing the text and language to the engine,
    # here we have marked slow=False. Which tells
    # the module that the converted audio should
    # have a high speed
    myobj = gTTS(text=mytext, lang=language, slow=False)

    # Saving the converted audio in a mp3 file named
    # welcome
    myobj.save("welcome.mp3")

    # Playing the converted file
    arguments = "..."
    proc = os.popen('xdg-open welcome.mp3')
    #time.sleep(1.5)
    #proc.terminate()
    #os.system("xdg-open welcome.mp3")
    #return_code = subprocess.call("./play_audio.sh")
    #print(return_code)
    quit()
    playsound("welcome.mp3")
    
    ####

    # files
    src = "welcome.mp3"
    dst = "welcome.wav"

    # convert wav to mp3
    sound = AudioSegment.from_mp3(src)
    play(sound)
    #sound.export(dst, format="wav")
    #getSound()


####
# Print out realtime audio volume as ascii bars

import sounddevice as sd
import numpy as np


import sys
import math
import numpy as np

def getSound():
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


    # b = S(5)
    # b.okay()
    # print(b.totalSound, b.totalCount)
    # noise = b.totalSound / b.totalCount
    # print("environment noise:", b.totalSound / b.totalCount)

    #import pydub
    #from pydub.playback import play

    #print(sys.argv[1])

    #song = AudioSegment.from_mp3("your_song.mp3")
    #song =  pydub.AudioSegment.from_file(file = wavFile, format = "wav")


    # boost volume by 6dB
    #louder_song = song + 6

    # reduce volume by 3dB
    #quieter_song = song - 3

    #Play song
    #play(louder_song)

    #save louder song
    #louder_song.export("louder_song.mp3", format='mp3')



    wavFile = "welcome.wav" #sys.argv[1]

    # import sounddevice as sd
    import soundfile as sf
    
    #print(noise)
    #weight = max(40, noise / 2.0)
    weight = 0.5
    print(weight)
    data, fs = sf.read(wavFile)
    print(data.shape)
    sd.play(weight*data, fs)
    sd.wait()

    # (fs1, x) = read('Traffic_stereo.wav', 'rb')
    # sd.play(x, fs1)
    # sd.wait()


stringtoSpeech("This is a test")
