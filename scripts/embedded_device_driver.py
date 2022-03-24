#!/usr/bin/env python
import serial
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import rospy
import math
import time


IN_FORCE = 0
IN_VELOCITY = 1
IN_UNDEF = 2


force_pub = rospy.Publisher('/suitbot/handle/force', Float64, queue_size=50)
twist_pub = rospy.Publisher('/suitbot/mobility/velocity', Twist, queue_size=50)


class SerialHandler:
    def __init__(self, port):
        self.ser = serial.Serial(port, 57600, timeout=1)
        self.ser.flush()
        self.stamp_counter = 0
        # keep sending start until teensy replies
        print("Trying to connect with teensy")
        #while self.readData() == False:
        self.ser.write("start".encode())
            #time.sleep(0.05)
        self.time_offset = 0 # machine time - device time
        print("Serial ready!")


    def readData(self):
        msg = None
        msg_type = IN_UNDEF
        line = self.ser.readline()
        if len(line) == 0:
            return msg, msg_type
        assert(self.time_offset != 0)
        values = line.rstrip().split('\t')
        assert(len(values) >= 3)
        secs = int(values[1])
        nsecs = int(values[2])
        stamp_in_machine_time = self.deviceTime2machineTime(rospy.Time(secs, nsecs))
        if values[0] == 'force':
            msg = Float64()
            msg.header.stamp = stamp_in_machine_time
            msg.header.frame_id = "force"
            msg.data = float(values[3])
            msg_type = IN_FORCE
        elif values[0] == 'encoder':
            msg = Twist()
            msg.header.stamp = stamp_in_machine_time
            msg.header.frame_id = "base_link"
            msg.linear.x = float(values[3])
            msg.linear.y = float(values[4])
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(values[5])
            msg_type = IN_VELOCITY
        else:
            print("Serial message " + values[0] + " not recognized")
        return msg, msg_type

    def deviceTime2machineTime(self, device_time):
        return device_time + self.time_offset


    def deviceDataInSec(self, data0, data1):
        secs = int(data0)
        nsecs = int(data1)
        t = rospy.Time(secs, nsecs)
        t_sec = t.to_sec()
        return t_sec

    def close(self):
        print("Trying to stop MCU...")
        t_stop = time.time()
        self.ser.write("stop".encode())
        # after SIGINT, wait 2 seconds to confirm hardware has stopped
        # while time.time()-t_stop < 2: 
        #     if self.readData() == STOP_RECEIVED:
        #         break
        self.ser.close()


    def initTimestamp(self):
        # discard frames in the beginning
        self.ser.flushOutput()
        self.ser.flushInput()
        for i in range(50):
            self.ser.readline()
        
        # do a cross time difference
        num_cross_td = 1
        time_offset = 0
        for i in range(num_cross_td):
            t_machine_1 = rospy.Time.now().to_sec()
            data1 = self.ser.readline().split('\t')
            data2 = self.ser.readline().split('\t')
            t_machine_2 = rospy.Time.now().to_sec()
        
            t_imu_1 = float(self.deviceDataInSec(data1[1], data1[2]))
            t_imu_2 = float(self.deviceDataInSec(data2[2], data2[2]))

            self.time_offset += (t_machine_1 - t_imu_1 + t_machine_2 - t_imu_2) / (2 * num_cross_td)
        # print("set time offset: ", time_offset)
        self.time_offset = rospy.Duration.from_sec(self.time_offset)


def loop():
    global port
    serial_handler = SerialHandler(port)
    serial_handler.initTimestamp()
    while not rospy.is_shutdown():
        msg, msg_type = serial_handler.readData()
        if msg_type == IN_FORCE:
            force_pub.publish(msg)
        elif msg_type == IN_VELOCITY:
            twist_pub.publish(msg)


if __name__ == '__main__':
    try:
        port = serial.Serial('/dev/ttyACM2', 115200)
    except:
        port = serial.Serial('/dev/ttyACM1', 115200)

    rospy.init_node('embedded_device_driver')
    loop()