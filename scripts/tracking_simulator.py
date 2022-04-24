#!/usr/bin/env python

# Modified from https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py

import imp
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import tf
import time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_about_axis
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistStamped, Vector3, PoseWithCovariance, TwistWithCovariance
from suitbot_ros.srv import SetCourse
from suitbot_ros.msg import TwoFloats

import sys, os.path
script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(script_dir)
from parameters import Parameters

# Parameters
k = 0.1  # look forward gain
Lfc = 1.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
Ki = 0.0  # speed integral gain
Kd = 0.0  # speed differential gain
dt = 0.1  # [s] time tick
WB = 0.23  # [m] wheel base of vehicle

show_animation = True

prev_error = 0.0
total_error = 0.0

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        # TODO if we only have the rear wheels
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    # for simulating the next state
    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x #- ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y #- ((WB / 2) * math.sin(self.yaw))
        
    # if we have the encoder reading
    def update_actual(self, v, w, d_t):
        self.v = v
        k00 = self.v * math.cos(self.yaw)
        k01 = self.v * math.sin(self.yaw)
        k02 = w 

        k10 = self.v * math.cos(self.yaw + k02*d_t/2.0)
        k11 = self.v * math.sin(self.yaw + k02*d_t/2.0)
        k12 = w

        k20 = k10
        k21 = k11
        k22 = w

        k30 = self.v * math.cos(self.yaw+d_t * k22)
        k31 = self.v * math.sin(self.yaw+d_t * k22)
        k32 = w

        self.x += d_t / 6.0 * (k00 + 2 * (k10 + k20) + k30)
        self.y += d_t / 6.0 * (k01 + 2 * (k11 + k21) + k31)
        self.yaw += d_t / 6.0 * (k02 + 2 * (k12 + k22) + k32)

        self.rear_x = self.x
        self.rear_y = self.y





    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def pid_control(target, current, dt):
    global prev_error, total_error
    cur_error = target - current
    a = Kp * (cur_error) + Kd * (cur_error - prev_error) / dt + Ki * total_error * dt
    prev_error = cur_error
    total_error += cur_error
    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


class TrackingSimulator:
    def __init__(self):
        self.counter = 0
        self.ctrl_pub = rospy.Publisher(parameters.ctrl_topic, Odometry, queue_size=10)
        #self.odom_pub = rospy.Publisher("/suitbot/odom", Odometry, queue_size=10)
        #self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        self.twist_sub = rospy.Subscriber(parameters.encoder_topic, TwistStamped, self.callback_update)
        self.path_service = rospy.Service(parameters.reset_path_service, SetCourse, self.callback_reset_course)
        self.force_sub = rospy.Subscriber(parameters.force_topic, TwoFloats, self.callback_force)
        self.target_course = None
        self.time = 0.0
        self.state = None
        if (parameters.manual_control == True or parameters.debug_odometry == True):
            self.state = State(x=parameters.init_x, y=parameters.init_y, yaw=parameters.init_theta, v=0.0)
        self.states = States()
        self.target_ind = None
        self.lastIndex = None
        self.dt = 0.1
        self.r = rospy.Rate(1.0/self.dt)
        self.target_speed = 0.6  # [m/s]
        self.T = 10000.0  # max simulation time
        self.t_prev = rospy.Time.now().to_sec()


    def callback_force(self, msg_in):
        force1 = msg_in.float1
        force2 = msg_in.float2
        # cap the target speed to (0, 1)
        self.target_speed = max(min(self.target_speed * ((force1 - force2) / 15.0 * 0.01 + 1.0), 0.75), 0)
        global Lfc
        # cap it to (1, 3)
        Lfc = max(1.0, 1.0 + (self.target_speed - 0.5) * 2 / 0.5)
        
        

    def callback_update(self, msg_in):
        try:
            v = msg_in.twist.linear.x
            w = msg_in.twist.angular.z
            t_cur = msg_in.header.stamp.secs + (msg_in.header.stamp.nsecs) / 1000000000.0
            d_t = t_cur - self.t_prev
            self.t_prev = t_cur
            self.state.update_actual(v, w, d_t)
            if (parameters.manual_control == True or parameters.debug_odometry == True):
                msg_out = Odometry()
                msg_out.header.stamp = msg_in.header.stamp
                pt = Point(self.state.x, self.state.y, 0)
                if (parameters.debug_odometry == True):
                    pt = Point(self.state.x - self.target_course.cx[0], self.state.y - self.target_course.cy[0], 0)
                # np array of x y z w
                q = quaternion_about_axis(self.state.yaw, (0,0,1))
                ang = Quaternion(q[0], q[1], q[2], q[3])
                msg_out.pose.pose = Pose(pt, ang)
                linear = Vector3(0.0, 0.0, 0.0)
                angular = Vector3(0.0, 0.0, 0.0)
                msg_out.twist.twist = Twist(linear, angular)
                self.ctrl_pub.publish(msg_out)
        except:
            rospy.logwarn_throttle_identical(5, "Tracking simulator: bad update_callback")

    def callback_reset_course(self, req):
        rospy.loginfo("Tracking simulator: Resetting course..")
        if (parameters.manual_control == True):
            return True
        l = len(req.points)
        cx = np.zeros(l)
        cy = np.zeros(l)
        for i in range(l):
            point = req.points[i]
            cx[i] = point.x
            cy[i] = point.y
        self.target_course = TargetCourse(cx, cy)

        path_cmd = req.path_cmd
        # initial state
        
        if path_cmd == 0:
            self.state = State(x=cx[0], y=cy[0], yaw=1.57, v=0.0)
        elif path_cmd == 1:
            self.state = State(x=cx[0], y=cy[0], yaw=-1.57, v=0.0)
        elif path_cmd == 2:
            self.state = State(x=cx[0], y=cy[0], yaw=3.14, v=0.0)

        #self.states.append(self.time, self.state)

        self.target_ind, _ = self.target_course.search_target_index(self.state)
        self.lastIndex = l - 1
        rospy.loginfo("Tracking simulator: Reset success")
        return True


    def loop(self):
        rospy.loginfo("Tracking simulator: entering loop")
        while parameters.manual_control == False and self.target_course == None and not rospy.is_shutdown():
            self.r.sleep()
        rospy.loginfo("Tracking simulator: start simulator")

       
        if (parameters.manual_control == False and parameters.debug_odometry == False): # not manually controlling
            t_init = rospy.Time.now().to_sec()
            t_cur = t_init
            while t_cur - t_init <= self.T and self.target_ind < self.lastIndex and not rospy.is_shutdown():
                # Calc control cmd
                ai = pid_control(self.target_speed, self.state.v, self.dt)
                di, self.target_ind = pure_pursuit_steer_control(
                    self.state, self.target_course, self.target_ind)

                # TODO add noise to the control
                #self.state.update(ai, di)  # Execute control and update vehicle state

                msg_out = Odometry()
                msg_out.header.stamp = rospy.Time.now()
                pt = Point(self.state.x, self.state.y, 0)
                # np array of x y z w
                q = quaternion_about_axis(self.state.yaw, (0,0,1))
                ang = Quaternion(q[0], q[1], q[2], q[3])
                msg_out.pose.pose = Pose(pt, ang)
                linear = Vector3(ai, 0.0, 0.0)
                angular = Vector3(0.0, 0.0, di)
                msg_out.twist.twist = Twist(linear, angular)
                self.ctrl_pub.publish(msg_out)
                
                self.r.sleep()
                t_cur = rospy.Time.now().to_sec()
                #self.states.append(t_cur, self.state)

            if self.lastIndex >= self.target_ind:
                msg_out = Odometry()
                pt = Point(self.state.x, self.state.y, 0)
                # np array of x y z w
                q = quaternion_about_axis(self.state.yaw, (0,0,1))
                ang = Quaternion(q[0], q[1], q[2], q[3])
                msg_out.pose.pose = Pose(pt, ang)
                linear = Vector3(0.0, 0.0, 0.0)
                angular = Vector3(0.0, 0.0, 0.0)
                msg_out.twist.twist = Twist(linear, angular)
                self.ctrl_pub.publish(msg_out)


if __name__ == '__main__':
    parameters = Parameters()
    parameters.initParameters()
    rospy.loginfo("Tracking simulator: node starting")
    rospy.init_node('tracking_simulator')
    simulator = TrackingSimulator()
    simulator.loop()
    rospy.loginfo("Tracking simulator: node going into spin")
    rospy.spin()
