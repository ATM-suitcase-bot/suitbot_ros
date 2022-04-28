/**
 * @file corner_extractor.cpp
 *
 * @brief Extract corners from rig image. The corners include checkerboard corners
 *        and the 4 outermost corners of the rig.
 *
 * @date 07/15/2021
 *
 * @author Tina Tian (yutian)
 */

#include "planner_node.h"
#include <opencv2/opencv.hpp>
#include "../utility/tic_toc.h"
#include <cmath>

PlannerNode::PlannerNode(ros::NodeHandle *nodehandle, parameters_t &_params)
    : nh(*nodehandle), params(_params)
{
    ROS_INFO("Initializing Planner Node...");
    if (map.initOccupancyGridMap(params.map_file, params.local_map_resolution) < 0)
    {
        ROS_ERROR("Failed to initialize Planner Node, exiting");
        // TODO error handling
        exit(1);
    }
    //map.coord_to_idx(goal_x, goal_y, goal_x_idx, goal_y_idx);
    initializeSubscribers();
    initializePublishers();
    planner_cli = nh.advertiseService(params.RESET_PLANNER_SERVICE, &PlannerNode::reset_planner, this);
}


void PlannerNode::set_start_indices(int x_idx_, int y_idx_)
{
    start_x_idx = x_idx_;
    start_y_idx = y_idx_;
}

void PlannerNode::set_goal_indices(int x_idx_, int y_idx_)
{
    goal_x_idx = x_idx_;
    goal_y_idx = y_idx_;
}


bool PlannerNode::reset_planner(suitbot_ros::ResetNodeRequest &req, suitbot_ros::ResetNodeResponse &res)
{
    path_cmd = 0;
    counter_cmd = 0;
    return true;
}



void PlannerNode::initializeSubscribers()
{
    ROS_INFO("Planner Node: Initializing Subscribers");
    odom_sub = nh.subscribe("/suitbot/odom", 1, &PlannerNode::subscriberCallback, this);
    ctrl_sub = nh.subscribe("/suitbot/odom_smooth", 1, &PlannerNode::controlCallback, this);
    waypoint_cli = nh.serviceClient<suitbot_ros::SetCourse>(params.RESET_PATH_SERVICE);

    path_cmd_sub = nh.subscribe(params.USR_CMD_TOPIC, 1, &PlannerNode::callback_path_cmd, this);
}

void PlannerNode::initializePublishers()
{
    ROS_INFO("Planner Node: Initializing Publishers");
    arrow_pub = nh.advertise<visualization_msgs::Marker>(params.PLANNER_ARROW_TOPIC, 1);
    planned_path_pub = nh.advertise<visualization_msgs::Marker>(params.PLANNED_PATH_TOPIC, 1);
    grid_map_pub = nh.advertise<visualization_msgs::MarkerArray>(params.GLOBAL_MAP_TOPIC, 1);
    initVisualization();
    if (params.publish_point_cloud_rate != 0)
    {
        grid_map_pub = nh.advertise<visualization_msgs::MarkerArray>(params.GLOBAL_MAP_TOPIC, 1);
        grid_map_pub_timer = nh.createTimer(ros::Duration(ros::Rate(params.publish_point_cloud_rate)),
                                                    &PlannerNode::publish2DMap, this);
    }
}

void PlannerNode::publish2DMap(const ros::TimerEvent&)
{
    ROS_DEBUG("[%s] Node::publishMapPointCloud()", ros::this_node::getName().data());
    grid_map_pub.publish(grid_map_marker_array);
}

void PlannerNode::controlCallback(const nav_msgs::Odometry &ctrl_in)
{
    this->pt = ctrl_in.pose.pose.position;

    float z = ctrl_in.pose.pose.orientation.z;
    float w = ctrl_in.pose.pose.orientation.w;
    this->yaw = std::atan2(2.0*(z*w), -1.0+2.0*w*w);

    //std::cout << "odom received" << this->pt << "  " << this->yaw << "\n";

    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.header.frame_id = params.global_frame_id;
    arrow_marker.ns = "planner_node";
    arrow_marker.id = 1000000;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose = ctrl_in.pose.pose;
    arrow_marker.pose.position.z = 1.0;
    arrow_marker.scale.x = 3;
    arrow_marker.scale.y = params.global_map_resolution;
    arrow_marker.scale.z = 1;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.1;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 1.0;
    arrow_pub.publish(arrow_marker);
}


void PlannerNode::subscriberCallback(const nav_msgs::Odometry &odom_in)
{
    // update robot pose in map
    //   convert odom into cell index
    int x_idx = 0, y_idx = 0;
    auto pt = odom_in.pose.pose.position;
    map.coord_to_idx(pt.x, pt.y, x_idx, y_idx);

    // replan
    if (a_star_planner() < 0)
    {
        ROS_WARN("Planner Node: No Path Found!");
    }

    // publish new plan
    updateVisualization();
}

void PlannerNode::publish_pose()
{
    geometry_msgs::TransformStamped pose;
    pose.transform.rotation.w = 1;
    pose.transform.rotation.x = 0;
    pose.transform.rotation.y = 0;
    pose.transform.rotation.z = 0;

    pose.transform.translation.x = 0;
    pose.transform.translation.y = 0;
    pose.transform.translation.z = 0;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.child_frame_id = "suitbot_frame";
    br.sendTransform(pose);
}

void PlannerNode::initVisualization()
{
    // Publish map as a marker array of cubes
    grid_map_marker_array.markers = vector<visualization_msgs::Marker>(map.num_occupied_cells + map.num_bloated_cells);

    int i = 0;
    ros::Time t_now = ros::Time::now();
    for (int r = 0; r < map.rows; r++)
    {
        for (int c = 0; c < map.cols; c++)
        {
            if (map.occupancy_map[r][c] < FREE)
            {
                grid_map_marker_array.markers[i].header.frame_id = "world";
                grid_map_marker_array.markers[i].ns = "planner_node";
                grid_map_marker_array.markers[i].type = visualization_msgs::Marker::CUBE;
                grid_map_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                grid_map_marker_array.markers[i].pose.position.x = params.global_map_resolution * r + params.global_map_resolution / 2;
                grid_map_marker_array.markers[i].pose.position.y = params.global_map_resolution  * c + params.global_map_resolution  / 2;
                grid_map_marker_array.markers[i].pose.position.z = 0;
                grid_map_marker_array.markers[i].pose.orientation.w = 1;
                grid_map_marker_array.markers[i].pose.orientation.x = 0;
                grid_map_marker_array.markers[i].pose.orientation.y = 0;
                grid_map_marker_array.markers[i].pose.orientation.z = 0;
                if (map.occupancy_map[r][c] == OCCUPIED)
                {
                    grid_map_marker_array.markers[i].color.r = 0.0;
                    grid_map_marker_array.markers[i].color.g = 1.0;
                    grid_map_marker_array.markers[i].color.b = 0.0;
                }
                else if (map.occupancy_map[r][c] == BLOATED)
                {
                    grid_map_marker_array.markers[i].color.r = 0.0;
                    grid_map_marker_array.markers[i].color.g = 0.0;
                    grid_map_marker_array.markers[i].color.b = 1.0;
                }
                grid_map_marker_array.markers[i].color.a = 0.75;
                grid_map_marker_array.markers[i].scale.x = params.global_map_resolution;
                grid_map_marker_array.markers[i].scale.y = params.global_map_resolution;
                grid_map_marker_array.markers[i].scale.z = params.global_map_resolution;
                grid_map_marker_array.markers[i].id = i;
                grid_map_marker_array.markers[i].header.stamp = t_now;
                i++;
            }
        }
    }
    grid_map_pub.publish(grid_map_marker_array);


    if (params.manual_control)
        return;
        
    planned_path_marker.type = visualization_msgs::Marker::LINE_LIST;
    planned_path_marker.action = visualization_msgs::Marker::ADD;
    planned_path_marker.pose.position.x = 0;
    planned_path_marker.pose.position.y = 0;
    planned_path_marker.pose.position.z = 1;
    planned_path_marker.pose.orientation.w = 1;
    planned_path_marker.pose.orientation.x = 0;
    planned_path_marker.pose.orientation.y = 0;
    planned_path_marker.pose.orientation.z = 0;
    planned_path_marker.scale.x = 0.3;
    planned_path_marker.color.r = 1;
    planned_path_marker.color.g = 0;
    planned_path_marker.color.b = 1;
    planned_path_marker.color.a = 1;
    planned_path_marker.header.frame_id = "world";
    planned_path_marker.ns = "planner_node";
}

void PlannerNode::updateVisualization()
{
    publish_pose();
    planned_path_marker.points.clear();

    if (params.manual_control)
        return;

    if (banked_steps.size() < 2)
        return;
    int p = 0;
    for (int i = 0; i < banked_steps.size() - 1; i++)
    {
        planned_path_marker.points.push_back(geometry_msgs::Point());
        planned_path_marker.points[p].x = banked_steps[i].first;
        planned_path_marker.points[p].y = banked_steps[i].second;
        planned_path_marker.points[p].z = 0;

        ++p;
        planned_path_marker.points.push_back(geometry_msgs::Point());
        planned_path_marker.points[p].x = banked_steps[i + 1].first;
        planned_path_marker.points[p].y = banked_steps[i + 1].second;
        planned_path_marker.points[p].z = 0;
        ++p;
    }
    planned_path_marker.header.stamp = ros::Time::now();
    planned_path_pub.publish(planned_path_marker);
}


void PlannerNode::callback_path_cmd(const std_msgs::Int32 &msg_in)
{
    if (counter_cmd == 0)
    {
        int cmd = msg_in.data;
        path_cmd = cmd; //removed -2 offset, using suitbot yaml index now
	ROS_WARN_STREAM("path replan cmd received");
	counter_cmd += 1;
    }
}


int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "planner_node");

    ros::NodeHandle nh;

    double goal_x, goal_y, start_x, start_y;
    ROS_INFO_STREAM("Planner Node: waiting for parameters to be set");

    parameters_t params;
    params.readParameters(nh);

    ROS_INFO_STREAM("Planner Node: Instantiating...");
    PlannerNode plannerNode(&nh, params);

    ros::Rate loop_rate(1);

    if (params.manual_control == false && params.use_audio == true)
    {
        while(ros::ok())
        {
            ROS_WARN_STREAM("main loop of planner running");
            if(plannerNode.path_cmd != 0 && !plannerNode.has_planned){

                // Read goal location from yaml- default start to elevators
                int x_idx = 0, y_idx = 0, x_goal_idx = 0, y_goal_idx = 0;
                start_x = plannerNode.pt.x;
                start_y = plannerNode.pt.y;

                //offset by 1 to fix missing index 1 in yaml
                x_goal_idx = (int)params.state_map[plannerNode.path_cmd-1][std::string("pos")][0];
                y_goal_idx = (int)params.state_map[plannerNode.path_cmd-1][std::string("pos")][1];
            
                plannerNode.map.coord_to_idx(start_x, start_y, x_idx, y_idx);
                std::cout << x_idx << ", " << y_idx << ", " << x_goal_idx << ", " << y_goal_idx << endl;

                plannerNode.set_start_indices(x_idx, y_idx);
                plannerNode.set_goal_indices(x_goal_idx, y_goal_idx);

                if (plannerNode.a_star_planner() < 0)
                {
                    ROS_WARN("Planner Node: No Path Found!");
                }

                suitbot_ros::SetCourse srv;
                srv.request.path_cmd = plannerNode.path_cmd;
                //Critical vis update to load in marker cords
                srv.request.path_cmd = plannerNode.path_cmd;
                plannerNode.updateVisualization();

                for (int i = 0; i < plannerNode.planned_path_marker.points.size(); i++)
                {
                    geometry_msgs::Point pt = plannerNode.planned_path_marker.points[i];
                    srv.request.points.push_back(pt);
                }
                if (plannerNode.waypoint_cli.call(srv)){
                    ROS_INFO("Reset course succeeded: %d", (int)srv.response.success);
                
                    plannerNode.counter_cmd = 1;
                    
		}
                else{
                    ROS_WARN("Failed to call service reset_course");
                }
                std::cout << "in planning, should scarcely execute\n";

            }

            ros::spinOnce();
            loop_rate.sleep();
        }
            
    }

    ros::spin();
    return 0;
}
