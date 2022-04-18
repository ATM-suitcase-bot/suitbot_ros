#!/usr/bin/env python3

  
from vosk import Model, KaldiRecognizer
import pyaudio
from difflib import SequenceMatcher



def findMatchingKeyword(keywords, inputSentence):
    userInput = inputSentence.split()
    userInput = userInput[3:-1]
    #print(userInput)
    D = {}
    highestWord = ""
    highestPercent = 0
    for word in userInput:
        if word not in D:
            D[word] = [0,""]
            percent, location = findMatchPercent(word, keywords)
            D[word][0] = percent
            D[word][1] = location
            if percent > highestPercent:
                highestPercent = percent
                highestWord = location


    return highestPercent, highestWord



def findMatchPercent(word, keywords):
    D = {}
    for key in keywords:
        D[key] = similar(key, word)

    location = max(D, key=D.get)
    percent = D[location]
    return percent, location


def similar(a, b):
    return SequenceMatcher(None, a, b).ratio()


def speechRecog():
    model = Model('./vosk-model-small-en-us-0.15')
    # read the model

    recognizer = KaldiRecognizer(model, 16000)


    keywords = ['yes', 'no', 'gates', 'hammerschlag', 'university center', 'home']
    # Recognize from the microphone
    cap = pyaudio.PyAudio()
    stream = cap.open(format=pyaudio.paInt16, channels = 1, rate = 16000, input = True, frames_per_buffer = 8192)
    stream.start_stream()

    while True:
        data = stream.read(4096) # 4 bytes
        # if len(data) == 0:
        #     break

        if recognizer.AcceptWaveform(data):
            input = recognizer.Result()
            print(input, '\n')
            similarity, matchedWord = findMatchingKeyword(keywords, input)
            print(similarity, matchedWord, '\n')
            if similarity > 0.5:
                break
   # print(keywords.index(matchedWord), '\n')
    return keywords.index(matchedWord)

#call this function:
speechRecog()
