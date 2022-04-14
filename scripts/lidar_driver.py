#!/usr/bin/env python

import imp
import rospy
import math
import time

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

from parameters import Parameters

class LidarDriver:
    def __init__(self):
        self.lidar_pub = rospy.Publisher(parameters.lidar_sync_topic, PointCloud2, queue_size=10)
        self.lidar_sub = rospy.Subscriber(parameters.lidar_original_topic, PointCloud2, self.callback_sync)


    def callback_sync(self, msg_in):
        msg_in.header.stamp = rospy.Time.now()
        self.lidar_pub.publish(msg_in)


if __name__ == '__main__':
    parameters = Parameters()
    parameters.initParameters()
    rospy.loginfo("Lidar Driver: node starting")
    rospy.init_node('lidar_driver')
    lidar_driver = LidarDriver()
    rospy.loginfo("lidar driver: node going into spin")
    rospy.spin()