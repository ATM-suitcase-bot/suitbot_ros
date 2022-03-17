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

using namespace std;

class PlannerNode
{
public:

    PlannerNode(ros::NodeHandle *nodehandle, string map_file, double goal_x_, double goal_y_); 


    int a_star_planner();

    void set_start_indices(int x_idx_, int y_idx_);

    void coord_to_idx(const double &coord_x, const double &coord_y, int &idx_x,  int &idx_y);

    void updateVisualization();

    void publish_pose();
    

    ros::NodeHandle nh; 
    ros::Subscriber odom_sub; 
    ros::Subscriber ctrl_sub;
    ros::ServiceClient waypoint_cli; // point array
    ros::Publisher arrow_pub;
    ros::Publisher  planned_path_pub; // line list
    visualization_msgs::Marker planned_path_marker;
    ros::Publisher grid_map_pub; // cubes
    visualization_msgs::MarkerArray grid_map_marker_array;

    tf2_ros::TransformBroadcaster br;

    double goal_x, goal_y, start_x, start_y;
    int goal_x_idx, goal_y_idx, start_x_idx, start_y_idx;

    double resolution = 0.5; // meter

    vector<vector<int>> occupancy_map; // <0 is occupied, 255 is free space, 100 is bloated
    int num_occupied_cells;


    vector<pair<double, double>> banked_steps;

    int rows = 0, cols = 0;


    int initOccupancyGridMap(string map_file);
    void initializeSubscribers(); 
    void initializePublishers();
    
    void subscriberCallback(const nav_msgs::Odometry &odom_in); 
    void controlCallback(const nav_msgs::Odometry &ctrl_in);

    // visualization
    void initVisualization();

    void bloat_obstacles();

    
    void get_successors(AugmentedNode &aug_node, vector<pair<int, int>> &successors);

    void compute_goals(unordered_set<int> &goal_set, int cur_idx);

    void compute_h(Node *node, Node *goal_node);

    double compute_cost(Node *n1, Node *n2);

    void backtrack(Node *goal_node);

}; 

#endif /* INITIALIZATION_H_ */