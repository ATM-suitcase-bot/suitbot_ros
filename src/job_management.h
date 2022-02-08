/**
 * @file job_management.h
 *
 * @brief Handles high-level job management of the robot
 *
 * @date 02/05/2022
 *
 * @author Tina Tian (yutian)  
 */

#ifndef JOB_MANAGEMENT_H_
#define JOB_MANAGEMENT_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 

#include <std_msgs/Bool.h> 
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h> 

#include <nav_msgs/Odometry.h>


#include "parameters.h"
#include <suitbot_ros/CmdMsg.h>
#include <suitbot_ros/InitializationSrvMsg.h>

#include "localization/initialization.h"


typedef enum state_num {
    IDLE,
    INITIALIZATION,
    GUIDING,
    ERROR
} state_t;

class JobManager
{
public:
    JobManager(ros::NodeHandle* nodehandle); 
    
private:
    ros::NodeHandle nh; 
    ros::Subscriber audio_sub;
    ros::Subscriber odom_sub; 
    ros::ServiceServer minimal_service_;
    ros::Publisher  minimal_publisher_;
    ros::ServiceClient initialization_cli;

    state_t state = IDLE;

    // TODO A planner node

    double goal_x, goal_y;


    
    double val_from_subscriber_; 
    double val_to_remember_;
    
    void initializeSubscribers(); 
    void initializePublishers();
    void initializeServices();

    void get_goal_coordinate(int goal, double &goal_x, double &goal_y);


    void audio_cmd_subscriber_callback(const suitbot_ros::CmdMsg &msg_in); 
    void localization_callback(const nav_msgs::Odometry::ConstPtr& msg_in);
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);


    int initialization_handler();
    int guiding_handler();
    int error_handler();
}; 

#endif /* JOB_MANAGEMENT_H_ */