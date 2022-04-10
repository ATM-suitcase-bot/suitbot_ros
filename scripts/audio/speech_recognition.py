

''' From https://github.com/ATM-suitcase-bot/Audio/blob/main/realTime.py'''

from vosk import Model, KaldiRecognizer, SetLogLevel
import pyaudio
from difflib import SequenceMatcher


def findMatchingKeyword(keywords_map, inputSentence):
    userInput = inputSentence.split()
    userInput = userInput[3:-1]
    #print(userInput)
    D = {}
    highestWord = ""
    highestPercent = 0
    for word in userInput:
        if word not in D:
            D[word] = [0,""]
            percent, location = findMatchPercent(word, keywords_map)
            D[word][0] = percent
            D[word][1] = location
            if percent > highestPercent:
                highestPercent = percent
                highestWord = location


    return highestPercent, highestWord



def findMatchPercent(word, keywords_map):
    D = {}
    for key in keywords_map:
        D[key] = similar(key, word)

    location = max(D, key=D.get)
    percent = D[location]
    return percent, location


def similar(a, b):
    return SequenceMatcher(None, a, b).ratio()


class SpeechRecognizer:
    def __init__(self, model_path_name):
        SetLogLevel(-1)
        self.model = Model(model_path_name)
        self.recognizer = KaldiRecognizer(self.model, 16000)


    # param in: keywords_map a list of keywords
    # return:   the keyword that matches the audio
    def speechRecog(self, keywords_map):
        # Recognize from the microphone
        cap = pyaudio.PyAudio()
        stream = cap.open(format=pyaudio.paInt16, channels = 1, rate = 16000, input = True, frames_per_buffer = 8192)
        stream.start_stream()

        while True:
            data = stream.read(4096) # 4 bytes
            # if len(data) == 0:
            #     break

            if self.recognizer.AcceptWaveform(data):
                input = self.recognizer.Result()
                #print(input, '\n')
                similarity, matchedWord = findMatchingKeyword(keywords_map, input)
                #print(similarity, matchedWord, '\n')
                if similarity > 0.5:
                    break
        # print(keywords.index(matchedWord), '\n')
        stream.stop_stream()
        stream.close()
        cap.terminate()
        return keywords_map.index[matchedWord]

