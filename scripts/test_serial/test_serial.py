#!/usr/bin/env python3
import serial
import time

string = "Python is interesting."

# string with encoding 'utf-8'
arr = bytes(string, 'utf-8')
print(arr)

ser = serial.Serial('/dev/ttyACM0', 9600)
ser.close()
ser.open()
#ser.flush()

def write_read(x):
    ser.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = ser.readline().rstrip()
    print("here", data)
    values_str = data.decode()
    values = values_str.split('\t')
    print("length: ", len(values))
    return values[0], values[1]

while True:
    num1 = input("Enter number1: ") # in python 3, num is a str, while in python 2, num is a float
    print(num1, type(num1))
    num2 = input("Enter number2: ")
    print(num2, type(num2))
    st = num1 + " " + num2
    value1, value2 = write_read(st)
    print(value1, type(value1), value2, type(value2)) # printing the value