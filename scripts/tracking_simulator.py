#!/usr/bin/env python

# Modified from https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py

import imp
import numpy as np
import math
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_about_axis
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistStamped, Vector3, PoseWithCovariance, TwistWithCovariance
from suitbot_ros.srv import SetCourse
from suitbot_ros.msg import TwoFloats
from suitbot_ros.msg import LocalMapMsg
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
import sys, os.path
script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(script_dir)
from parameters import Parameters
from local_planner import PathPerturb
import cv2

# Parameters
k = 0.1  # look forward gain
Lfc = 1.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain

kernel = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]).astype(np.uint8)

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        
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

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)


#Actually just p control- is pretty good for path tracking
def pid_control(target, current, dt):
    cur_error = target - current
    a = Kp * (cur_error)
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
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
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
        #print(ind, len(self.cx))
        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind, override_pt):
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

    if(not override_pt is None): #obstacle avoidance overrides other path following
        tx = override_pt[0]
        ty = override_pt[1]

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    isflip = np.abs(alpha) > (np.pi/2)

    delta = math.atan2(2.0 * 0.25 * math.sin(alpha) / Lf, 1.0)

    return delta, ind, isflip


class TrackingSimulator:
    def __init__(self):
        self.counter = 0
        self.ctrl_pub = rospy.Publisher(parameters.ctrl_topic, Odometry, queue_size=10)
        self.obs_pub = rospy.Publisher("/suitbot/obs_py_im", Image, queue_size=10)
        self.drive_status_pub = rospy.Publisher(parameters.drive_status_topic, Int8, queue_size=10)
        
        self.twist_sub = rospy.Subscriber(parameters.encoder_topic, TwistStamped, self.callback_update)
        self.path_service = rospy.Service(parameters.reset_path_service, SetCourse, self.callback_reset_course)
        self.force_sub = rospy.Subscriber(parameters.force_topic, TwoFloats, self.callback_force)
        
        self.obs_sub = rospy.Subscriber("/suitbot/local_obs", LocalMapMsg, self.callback_obs)

        self.target_course = None
        self.time = 0.0
        #self.state = None
        #if (parameters.manual_control == True or parameters.debug_odometry == True):
        self.state = State(x=parameters.init_x, y=parameters.init_y, yaw=parameters.init_theta, v=0.0)
        self.target_ind = None
        self.lastIndex = None
        self.dt = 0.1
        self.r = rospy.Rate(1.0/self.dt)
        self.target_speed = 0.6  # [m/s]
        self.t_prev = rospy.Time.now().to_sec()

        #init path perturation object for path recalculation
        self.path_perturb = PathPerturb()
        self.avoiding = False
        self.target_pt = None
        self.has_spun = True #has the robot done a lil localization loop
        self.spin_v = [0.12, 0.45] #v/omega to spin with
        self.spin_time = 18.0 #time (s) to spin
        self.spin_start = rospy.Time.now().to_sec()

    def get_local(self, index):
        #render goal pixel in the robot's space
        relative_x = self.target_course.cx[index] - self.state.x
        relative_y = self.target_course.cy[index] - self.state.y
        [local_x, local_y] = self.path_perturb.rot_point([relative_x, relative_y], -1*self.state.yaw)
        return [local_x, local_y]

    #Callback on obstacle avoidance (local)
    def callback_obs(self, msg_in):
        #read input message to python-style array form
        if(self.target_course is None):
            return
        im_dims = [msg_in.rows, msg_in.cols]
        robot_pos = [msg_in.robot_x_idx, msg_in.robot_y_idx]

        if(robot_pos[0] >= im_dims[0] or robot_pos[1] >= im_dims[1]):
            print('map does not contain robot position, invalid replanning')
            return
        
        raw_arr = np.reshape(msg_in.cells, im_dims)
        raw_arr = raw_arr.astype(np.uint8)
        raw_arr = cv2.erode(raw_arr, kernel, iterations=1)
        occ_map = (raw_arr == 0).astype(int) #save a simple boolean map form of the input array
        
        occ_map[robot_pos[0]-1:robot_pos[0]+2, robot_pos[1]-1:robot_pos[1]+2] = 0
        
        raw_arr = (255*occ_map.astype(int)).astype(np.uint8)
        raw_arr = np.stack([raw_arr, raw_arr, raw_arr], axis=2)

        #convert goal point into pixel in local map
        [local_x, local_y] = self.get_local(self.target_ind)
        [pix_x, pix_y] = self.path_perturb.get_pix_ind([local_x, local_y], robot_pos)

        alt_endpoint = False
        alt_offset = 0
        
        #check if initial path yields collision
        while(alt_offset < 3 and occ_map[pix_x, pix_y] and self.target_ind < len(self.target_course.cx)-1-alt_offset):
            print('old goal point is in a wall')
            alt_offset += 1
            [local_x, local_y] = self.get_local(self.target_ind+alt_offset)
            [pix_x, pix_y] = self.path_perturb.get_pix_ind([local_x, local_y], robot_pos)
            alt_endpoint = True
        
        fine_bot_pos = np.array([self.state.x, self.state.y, 0.0])
         
        fine_goal_pos = fine_bot_pos[0:2] + np.array([local_x, local_y])
        bot_pix = self.path_perturb.get_pix_ind(fine_bot_pos, [0, 0])
        maps_offset = np.array([bot_pix[0]-robot_pos[0], bot_pix[1]-robot_pos[1]])
        init_path_safe = self.path_perturb.check_path(occ_map, maps_offset, fine_bot_pos, fine_goal_pos, None)

        if(not init_path_safe):
            
            best_target = self.path_perturb.get_better_path(fine_bot_pos, fine_goal_pos, occ_map, maps_offset)

            if(best_target is None):
                print('replanning failed, need new global path OR to wait')
                #need to handle logic here- should probably halt control, maybe send signal to planner
            else:
                best_target_local_vec = np.array(best_target) - fine_bot_pos[0:2]
                best_target_offs = np.array(self.path_perturb.rot_point(best_target_local_vec, self.state.yaw))
                best_target_global = best_target_offs + fine_bot_pos[0:2]
                

                #then we go to the 'best_target_global'
                self.avoiding = True
                self.target_pt = best_target_global
        else: #init path found to be safe
            self.avoiding = False
            self.target_pt = None

            if(alt_endpoint):
                self.avoiding = True
                self.target_pt = [self.target_course.cx[index+alt_offset], self.target_course.cy[index+alt_offset]]
        
        #render key pixels on the published local map
        raw_arr[robot_pos[0], robot_pos[1], :] = [0, 0, 255] #robot is red
        raw_arr[pix_x, pix_y, :] = [0, 255, 0] #goal is green
        if(not init_path_safe):
            raw_arr[0, :, 2] = 255

        self.obs_pub.publish(CvBridge().cv2_to_imgmsg(np.flip(np.flip(raw_arr, axis=0), axis=1)))

    #Callback on force feedback
    def callback_force(self, msg_in):
        force1 = msg_in.float1
        force2 = msg_in.float2
        # cap the target speed to (0, 1)
        self.target_speed = max(min(self.target_speed * ((force1 - force2) / 15.0 * 0.01 + 1.0), 0.75), 0)
        global Lfc
        # cap it to (1, 3)
        Lfc = max(1.0, 1.0 + (self.target_speed - 0.5) * 2 / 0.5)

    #Callback on odometry
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
                #print(self.state.yaw)
                ang = Quaternion(q[0], q[1], q[2], q[3])
                msg_out.pose.pose = Pose(pt, ang)
                linear = Vector3(0.0, 0.0, 0.0)
                angular = Vector3(0.0, 0.0, 0.0)
                msg_out.twist.twist = Twist(linear, angular)
                self.ctrl_pub.publish(msg_out)
        except:
            rospy.logwarn_throttle_identical(5, "Tracking simulator: bad update_callback")

    #Callback on replanning, new course
    def callback_reset_course(self, req):
        rospy.loginfo("Tracking simulator: Resetting course..")
        if (parameters.manual_control == True):
            return True
        l = len(req.points)
        print(l)
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

        self.target_ind, _ = self.target_course.search_target_index(self.state)
        self.lastIndex = l - 1
        rospy.loginfo("Tracking simulator: Reset success")
        return True

    def getOdoOut(self, ai, di):
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
        return msg_out

    def loop(self):
        rospy.loginfo("Tracking simulator: entering loop")
        while parameters.manual_control == False and self.target_course == None and not rospy.is_shutdown():
            if(self.has_spun):
                self.ctrl_pub.publish(self.getOdoOut(0.0, 0.0))
            else:
                if(rospy.Time.now().to_sec() - self.spin_start > self.spin_time):
                    self.has_spun = True
                self.ctrl_pub.publish(self.getOdoOut(self.spin_v[0], self.spin_v[1]))
            self.r.sleep()
        
        rospy.loginfo("Tracking simulator: start simulator")
       
        if (parameters.manual_control == False and parameters.debug_odometry == False): # not manually controlling
            t_init = rospy.Time.now().to_sec()
            t_cur = t_init
            while not rospy.is_shutdown():
                if(self.target_ind < self.lastIndex):
                    # Calc control cmd
                    ai = pid_control(self.target_speed, self.state.v, self.dt)
                    di, self.target_ind, isflip = pure_pursuit_steer_control(
                        self.state, self.target_course, self.target_ind, self.target_pt)

                    if(isflip):
                        self.ctrl_pub.publish(self.getOdoOut(ai/2.0, di))

                    else:
                        self.ctrl_pub.publish(self.getOdoOut(ai, di))

                else: #stopping
                    self.ctrl_pub.publish(self.getOdoOut(0.0, 0.0))
                    status_msg = Int8()
                    status_msg.data = np.int8(2)
                    self.drive_status_pub.publish(status_msg)
                    
                self.r.sleep()
                t_cur = rospy.Time.now().to_sec()


if __name__ == '__main__':
    parameters = Parameters()
    parameters.initParameters()
    rospy.loginfo("Tracking simulator: node starting")
    rospy.init_node('tracking_simulator')
    simulator = TrackingSimulator()
    simulator.loop()
    rospy.loginfo("Tracking simulator: node going into spin")
    rospy.spin()
