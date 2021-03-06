/**
 * @file planner_node.h
 *
 * @brief 
 *
 * @date 02/10/2021
 *
 * @author Tina Tian (yutian)  
 */

#ifndef PLANNER_NODE_H_
#define PLANNER_NODE_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"

#include <std_msgs/Int32.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

#include "a_star.h"
//#include <Eigen/Dense>
#include <chrono>
#include "suitbot_ros/SetCourse.h"
#include "suitbot_ros/ResetNode.h"

#include "../parameters.h"
#include "../occupancy_map.h"
#include "../utility/utils.h"

using namespace std;

class PlannerNode
{
public:

    PlannerNode(ros::NodeHandle *nodehandle, parameters_t &_params); 


    int a_star_planner();

    void set_start_indices(int x_idx_, int y_idx_);

    void set_goal_indices(int x_idx_, int y_idx_);


    void updateVisualization();

    void publish_pose();
    
    parameters_t params;

    ros::NodeHandle nh; 
    ros::Subscriber odom_sub; 
    ros::Subscriber ctrl_sub;
    ros::ServiceClient waypoint_cli; // point array
    ros::ServiceServer planner_cli;
    ros::Publisher arrow_pub;
    ros::Publisher  planned_path_pub; // line list
    visualization_msgs::Marker planned_path_marker;
    
    ros::Publisher grid_map_pub; // cubes
    visualization_msgs::MarkerArray grid_map_marker_array;
    ros::Timer grid_map_pub_timer;

    tf2_ros::TransformBroadcaster br;

    ros::Subscriber path_cmd_sub;

    double goal_x, goal_y, start_x, start_y;
    int goal_x_idx, goal_y_idx, start_x_idx, start_y_idx;


    OccupancyMap map;

    vector<pair<double, double>> banked_steps;


    int path_cmd = 0;
    int counter_cmd = 0;

    //Store the most recently observed odometry
    geometry_msgs::Point pt;
    double yaw;


    void initializeSubscribers(); 
    void initializePublishers();
    
    void subscriberCallback(const nav_msgs::Odometry &odom_in); 
    void controlCallback(const nav_msgs::Odometry &ctrl_in);

    void callback_path_cmd(const std_msgs::Int32 &msg_in);

    void publish2DMap(const ros::TimerEvent&);

    bool reset_planner(suitbot_ros::ResetNodeRequest &req, suitbot_ros::ResetNodeResponse &res);
    // visualization
    void initVisualization();

    
    void get_successors(AugmentedNode &aug_node, vector<pair<int, int>> &successors);

    void compute_goals(unordered_set<int> &goal_set, int cur_idx);

    void compute_h(Node *node, Node *goal_node);

    double compute_cost(Node *n1, Node *n2);

    void backtrack(Node *goal_node);

    bool has_planned = false;

};


#endif /* INITIALIZATION_H_ */
