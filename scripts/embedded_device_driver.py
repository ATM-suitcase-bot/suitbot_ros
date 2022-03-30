#!/usr/bin/env python
import serial
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistStamped, Vector3, PoseWithCovariance, TwistWithCovariance
import rospy
import math
import time


IN_FORCE = 0
IN_VELOCITY = 1
IN_UNDEF = 2
STOP_RECEIVED = 3



class SerialHandler:
    def __init__(self, port):
        self.ser = port
        self.ser.flush()
        self.stamp_counter = 0
        self.ctrl_sub = rospy.Subscriber("/suitbot/ctrl/velocity", Odometry, self.callback_ctrl)
        self.force_pub = rospy.Publisher('/suitbot/handle/force', Float64, queue_size=50)
        self.twist_pub = rospy.Publisher('/suitbot/mobility/velocity', TwistStamped, queue_size=50)
        # keep sending start until teensy replies
        #print("Trying to connect with MCU")
        #while self.readData() == False:
        #self.ser.write("start".encode())
            #time.sleep(0.05)
        self.time_offset = 0 # machine time - device time
        #print("Serial ready!")
        self.r = rospy.Rate(5)

    def callback_ctrl(self, msg_in):
        ai = msg_in.twist.twist.linear.x
        di = msg_in.twist.twist.angular.z
        speed_str = str(ai)
        print("speed cmd from nuc: ", speed_str)
        ang_str = str(di)
        msg_to_mcu = speed_str + "\t" + ang_str 
        self.ser.write(msg_to_mcu.encode())


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
            msg = TwistStamped()
            msg.header.stamp = stamp_in_machine_time
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = float(values[3])
            print("encoder velocity from mcu: ", values[3])
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = float(values[4])
            msg_type = IN_VELOCITY
        elif values[0] == 'data':
            print("mcu received comand then sent back: ", values[3])
        elif msg[0] == 'stopping':
            print('MCU stops')
            return MSG, STOP_RECEIVED
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
        self.ser.write("0\t0".encode())
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


    def loop(self):
        self.initTimestamp()
        while not rospy.is_shutdown():
            msg, msg_type = self.readData()
            if msg_type == IN_FORCE:
                self.force_pub.publish(msg)
            elif msg_type == IN_VELOCITY:
                self.twist_pub.publish(msg)
            self.r.sleep()

if __name__ == '__main__':
    try:
        port = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
    except:
        port = serial.Serial('/dev/ttyACM1', 57600, timeout=1)
    rospy.init_node('embedded_device_driver')
    serial_handler = SerialHandler(port)
    serial_handler.loop()