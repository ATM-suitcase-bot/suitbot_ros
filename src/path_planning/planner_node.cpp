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

#define OCCUPIED 0
#define FREE 255
#define BLOATED 100

PlannerNode::PlannerNode(ros::NodeHandle *nodehandle, parameters_t &_params)
    : nh(*nodehandle), params(_params)
{
    ROS_INFO("Initializing Planner Node...");
    if (initOccupancyGridMap(params.map_file) < 0)
    {
        ROS_ERROR("Failed to initialize Planner Node, exiting");
        // TODO error handling
    }
    
    rows = occupancy_map.size();
    cols = occupancy_map[0].size();
    bloat_obstacles();
    coord_to_idx(goal_x, goal_y, goal_x_idx, goal_y_idx);
    initializeSubscribers();
    initializePublishers();
}

void PlannerNode::coord_to_idx(const double &coord_x, const double &coord_y, int &idx_x, int &idx_y)
{
    // TODO
    idx_x = (int)(coord_x / resolution);
    idx_y = (int)(coord_y / resolution);
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

int PlannerNode::initOccupancyGridMap(string map_file)
{

    /*
        // Store map into vector
        std::ifstream in_stream;
        in_stream.open(map_file);
        if (in_stream.is_open())
        {
            string line;
            while (getline(in_stream, line))
            {
                vector<double> vec_line;
                std::stringstream line_stream(line);
                double element; // probability between 0 and 1
                while (line_stream >> element)
                {
                    vec_line.push_back(element);
                }
                occupancy_map.push_back(vec_line);
            }
            in_stream.close();
        }
    */
    cv::Mat map_img = cv::imread(map_file, cv::IMREAD_GRAYSCALE);
    //cv::imshow("map", map_img);
    //cv::waitKey(0);
    if (!map_img.empty())
    {
        for (int r = 0; r < map_img.rows; r++)
        {
            vector<int> line;
            for (int c = 0; c < map_img.cols; c++)
            {
                // outimg.at<uchar>(239 - y_coord , x_coord) = 0;
                int num = (map_img.at<uchar>(r, c) < 50) ? 0 : 255;
                line.push_back(num);
            }
            occupancy_map.push_back(line);
        }
    }
    else
    {
        ROS_ERROR("No such map file named %s", map_file.c_str());
        return -1;
    }
    return 0;
}

void PlannerNode::bloat_obstacles()
{
    int count = 0;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (occupancy_map[r][c] == 0)
            {
                count ++;
                // check all directions
                for (int i = -1; i < 2; i++)
                {
                    int row_new = r + i;
                    for (int j = -1; j < 2; j++)
                    {
                        int col_new = c + j;
                        if ((row_new != r || col_new != c) && row_new >= 0 && row_new < rows && col_new >= 0 && col_new < cols)
                        {
                            if (occupancy_map[row_new][col_new] == 255)
                            {
                                occupancy_map[row_new][col_new] = 100;
                                
                            }
                            
                        }
                    }
                }
            }
        }
    }
    std::cout << count << std::endl;
}

void PlannerNode::initializeSubscribers()
{
    ROS_INFO("Planner Node: Initializing Subscribers");
    odom_sub = nh.subscribe("/suitbot/odom", 1, &PlannerNode::subscriberCallback, this);
    ctrl_sub = nh.subscribe(params.CTRL_TOPIC, 1, &PlannerNode::controlCallback, this);
    waypoint_cli = nh.serviceClient<suitbot_ros::SetCourse>(params.RESET_PATH_SERVICE);

    path_cmd_sub = nh.subscribe(params.USR_CMD_TOPIC, 1, &PlannerNode::callback_path_cmd, this);
}

void PlannerNode::initializePublishers()
{
    ROS_INFO("Planner Node: Initializing Publishers");
    initVisualization();
    arrow_pub = nh.advertise<visualization_msgs::Marker>(params.PLANNER_ARROW_TOPIC, 1);
}


void PlannerNode::controlCallback(const nav_msgs::Odometry &ctrl_in)
{
    //geometry_msgs::Point pt = ctrl_in.pose.pose.position;
    //double yaw = ctrl_in.pose.pose.orientation;
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.header.frame_id = "world";
    arrow_marker.ns = "planner_node";
    arrow_marker.id = 1000000;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose = ctrl_in.pose.pose;
    arrow_marker.scale.x = 6;
    arrow_marker.scale.y = 0.5;
    arrow_marker.scale.z = 2;
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
    coord_to_idx(pt.x, pt.y, x_idx, y_idx);

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
    // count num of occupied cells
    int count = 0;
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (occupancy_map[r][c] < FREE)
                count++;
        }
    }
    num_occupied_cells = count;

    grid_map_pub = nh.advertise<visualization_msgs::MarkerArray>(params.GLOBAL_MAP_TOPIC, 1);
    grid_map_marker_array.markers = vector<visualization_msgs::Marker>(count);

    int i = 0;
    ros::Time t_now = ros::Time::now();
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (occupancy_map[r][c] < FREE)
            {
                grid_map_marker_array.markers[i].header.frame_id = "world";
                grid_map_marker_array.markers[i].ns = "planner_node";
                grid_map_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                grid_map_marker_array.markers[i].pose.position.x = resolution * c + resolution / 2;
                grid_map_marker_array.markers[i].pose.position.y = resolution * r + resolution / 2;
                grid_map_marker_array.markers[i].pose.position.z = 0;
                grid_map_marker_array.markers[i].pose.orientation.w = 1;
                grid_map_marker_array.markers[i].pose.orientation.x = 0;
                grid_map_marker_array.markers[i].pose.orientation.y = 0;
                grid_map_marker_array.markers[i].pose.orientation.z = 0;
                if (occupancy_map[r][c] == OCCUPIED)
                {
                    grid_map_marker_array.markers[i].color.r = 0.0;
                    grid_map_marker_array.markers[i].color.g = 1.0;
                    grid_map_marker_array.markers[i].color.b = 0.0;
                }
                else if (occupancy_map[r][c] == BLOATED)
                {
                    grid_map_marker_array.markers[i].color.r = 0.0;
                    grid_map_marker_array.markers[i].color.g = 0.0;
                    grid_map_marker_array.markers[i].color.b = 1.0;
                }
                grid_map_marker_array.markers[i].color.a = 0.75;
                grid_map_marker_array.markers[i].scale.x = 0.5;
                grid_map_marker_array.markers[i].scale.y = 0.5;
                grid_map_marker_array.markers[i].scale.z = 0.5;
                grid_map_marker_array.markers[i].id = i;
                grid_map_marker_array.markers[i].header.stamp = t_now;
                i++;
            }
        }
    }
    grid_map_pub.publish(grid_map_marker_array);

    planned_path_pub = nh.advertise<visualization_msgs::Marker>(params.PLANNED_PATH_TOPIC, 1);
    planned_path_marker.type = visualization_msgs::Marker::LINE_LIST;
    planned_path_marker.action = visualization_msgs::Marker::ADD;
    planned_path_marker.pose.position.x = 0;
    planned_path_marker.pose.position.y = 0;
    planned_path_marker.pose.position.z = 0;
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
    grid_map_pub.publish(grid_map_marker_array);
    planned_path_marker.points.clear();
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
        path_cmd = cmd - 2;
        counter_cmd += 1;
    }
}


int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "planne_node");

    ros::NodeHandle nh;

    double goal_x, goal_y, start_x, start_y;
    ROS_INFO_STREAM("Planner Node: waiting for parameters to be set");

    parameters_t params;
    params.readParameters(nh);

    ROS_INFO("Planner Node: Instantiating...");
    PlannerNode plannerNode(&nh, params);

    ros::Rate loop_rate(1);

    // left mid right
    // mid: first turns right, then go straight, then turn left
    double start_xs[3] = {23.5, 27.0, 30.0}; // x idx = 50   46
    double start_ys[3] = {64.0, 87.5, 71.0}; // y idx = 31   132
    double goal_xs[3] = {30.0, 27.0, 23.5};
    double goal_ys[3] = {71.0, 75.0, 64.0};

    while (plannerNode.path_cmd == -1 && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        continue;
    }

    std::cout << "plan idx: " << plannerNode.path_cmd << std::endl;

    // for testing purpose
    int x_idx = 0, y_idx = 0, x_goal_idx = 0, y_goal_idx = 0;
    start_x = start_xs[plannerNode.path_cmd];
    start_y = start_ys[plannerNode.path_cmd];
    goal_x = goal_xs[plannerNode.path_cmd];
    goal_y = goal_ys[plannerNode.path_cmd];
    cout << start_x << ", " << start_y << ", " << goal_x << ", " << goal_y << endl;

    plannerNode.coord_to_idx(start_x, start_y, x_idx, y_idx);
    plannerNode.coord_to_idx(goal_x, goal_y, x_goal_idx, y_goal_idx);
    plannerNode.set_start_indices(x_idx, y_idx);
    plannerNode.set_goal_indices(x_goal_idx, y_goal_idx);

    bool no_course = true;

    // replan
    TicToc t;
    if (plannerNode.a_star_planner() < 0)
    {
        ROS_WARN("Planner Node: No Path Found!");
    }
    ROS_INFO("Planning time: %f ms", t.toc());

    while (ros::ok())
    {
        plannerNode.updateVisualization();
        if (no_course) 
        {
            suitbot_ros::SetCourse srv;
            srv.request.path_cmd = plannerNode.path_cmd;
            for (int i = 0; i < plannerNode.planned_path_marker.points.size(); i++)
            {
                geometry_msgs::Point pt = plannerNode.planned_path_marker.points[i];
                srv.request.points.push_back(pt);
            }
            if (plannerNode.waypoint_cli.call(srv))
            {
                ROS_INFO("Reset course succeedded: %d", (int)srv.response.success);
            }
            else
            {
                ROS_WARN("Failed to call service reset_course");
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
            no_course = false;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    // ROS_INFO("Planner Node: main going into spin");
    // ros::spin();
    return 0;
}