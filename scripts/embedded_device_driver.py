#!/usr/bin/env python3
import serial
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistStamped, Vector3, PoseWithCovariance, TwistWithCovariance
from suitbot_ros.msg import TwoFloats
import rospy
import math
import time
from threading import Thread, Lock


IN_FORCE = 0
IN_VELOCITY = 1
IN_UNDEF = 2
STOP_RECEIVED = 3

port = '/dev/ttyACM0'

class SerialHandler:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.ser.flush()
        self.ser.close()
        self.ser.open()
        self.stamp_counter = 0
        self.ctrl_sub = rospy.Subscriber("/suitbot/ctrl/velocity", Odometry, self.callback_ctrl)
        self.force_pub = rospy.Publisher('/suitbot/handle/force', TwoFloats, queue_size=50)
        self.twist_pub = rospy.Publisher('/suitbot/mobility/velocity', TwistStamped, queue_size=50)
        self.time_offset = None # machine time - device time
        #self.initTimestamp()
        print("offset: ", self.time_offset)
        print("Serial ready!")
        self.r = rospy.Rate(50)

    def callback_ctrl(self, msg_in):
        ai = msg_in.twist.twist.linear.x
        di = msg_in.twist.twist.angular.z
        ai_str = str(round(ai, 4))
        di_str = str(round(di, 4))
        msg_to_mcu = ai_str + "\t" + di_str
        self.write_data(msg_to_mcu)

    def write_data(self, str_out):
        bts = bytes(str_out, 'utf-8')
        self.ser.write(bts)

    def readData(self):
        msg = None
        msg_type = IN_UNDEF
        line = self.ser.readline().rstrip()
        if len(line) == 0: # empty line
            return msg, msg_type
        values_str = line.decode()
        #print(values_str)
        values = values_str.split('\t')
        #assert(self.time_offset != None)
        assert(len(values) > 3)
        # format: msg_type, secs, nsecs, value0, value1, ..
        secs = int(values[1])
        nsecs = int(values[2])
        stamp_in_machine_time = rospy.Time.now() #self.deviceTime2machineTime(rospy.Time(secs, nsecs))
        if values[0] == 'force':
            msg = TwoFloats()
            msg.header.stamp = stamp_in_machine_time
            msg.header.frame_id = "force"
            msg.float1 = float(values[3]) # nonzero if push
            msg.float2 = float(values[4]) # nonzero if pull
            print("force from mcu: %.4f, %.4f" % (msg.float1, msg.float2))
            msg_type = IN_FORCE
        elif values[0] == 'encoder':
            msg = TwistStamped()
            msg.header.stamp = stamp_in_machine_time
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = float(values[3])
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = float(values[4])
            print("encoder velocity from mcu: %.4f, %.4f" % (msg.twist.linear.x, msg.twist.angular.z))
            msg_type = IN_VELOCITY
        elif values[0] == 'voltage':
            voltage = float(values[3])
            print('voltage: %.4f' % (voltage))
            return msg, IN_UNDEF
        elif values[0] == 'data':
            v1 = float(values[3])
            v2 = float(values[4])
            #print("cmd sending back to pc from mcu: %.8f, %.8f" % (v1, v2))
            return msg, IN_UNDEF
        elif values[0] == 'stopping':
            print('MCU stops')
            return msg, STOP_RECEIVED
        else:
            print("Serial message " + values[0] + " not recognized")
        return msg, msg_type

    def deviceTime2machineTime(self, device_time):
        return device_time + self.time_offset # currently not adding time offset


    def deviceDataInSec(self, data0, data1):
        secs = int(data0)
        nsecs = int(data1)
        t = rospy.Time(secs, nsecs)
        t_sec = t.to_sec()
        return t_sec

    def close(self):
        print("Trying to stop MCU...")
        t_stop = time.time()
        self.write_data("0.0\t0.0")
        # after SIGINT, wait 2 seconds to confirm hardware has stopped
        while time.time()-t_stop < 1:
            pass 
        #     if self.readData() == STOP_RECEIVED:
        #         break
        self.ser.close()


    def initTimestamp(self):
        # discard frames in the beginning
        self.ser.flushOutput()
        self.ser.flushInput()
        for i in range(10):
            self.ser.readline()
        
        # do a cross time difference
        num_cross_td = 1
        for i in range(num_cross_td):
            t_machine_1 = rospy.Time.now().to_sec()
            data1 = self.ser.readline().rstrip().decode().split('\t')
            data2 = self.ser.readline().rstrip().decode().split('\t')

            t_machine_2 = rospy.Time.now().to_sec()
        
            t_imu_1 = float(self.deviceDataInSec(data1[1], data1[2]))
            t_imu_2 = float(self.deviceDataInSec(data2[2], data2[2]))

            self.time_offset += (t_machine_1 - t_imu_1 + t_machine_2 - t_imu_2) / (2 * num_cross_td)
        # print("set time offset: ", time_offset)
        self.time_offset = rospy.Duration.from_sec(self.time_offset)


    def loop(self):
        #self.initTimestamp()
        while not rospy.is_shutdown():
            msg, msg_type = self.readData()
            if msg_type == IN_FORCE:
                self.force_pub.publish(msg)
                self.r.sleep()
            elif msg_type == IN_VELOCITY:
                self.twist_pub.publish(msg)
                self.r.sleep()
        self.close()

def serialLoop():
    global port
    serial_handler = SerialHandler(port)
    while not rospy.is_shutdown():
        msg, msg_type = serial_handler.readData()
        if msg_type == IN_FORCE:
            serial_handler.force_pub.publish(msg)
        elif msg_type == IN_VELOCITY:
            serial_handler.twist_pub.publish(msg)
            #self.r.sleep()
    serial_handler.close()


if __name__ == '__main__':
    rospy.init_node('embedded_device_driver')
    t_stamp = Thread(target=serialLoop)
    t_stamp.start()
    rospy.spin()
    t_stamp.join()
