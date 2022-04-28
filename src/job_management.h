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
#include <unistd.h>
#include <string>
#include <vector>

#include <ros/ros.h> 

#include <std_msgs/Bool.h> 
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h> 


#include <nav_msgs/Odometry.h>

#include <std_srvs/SetBool.h>


#include "parameters.h"

#include <suitbot_ros/InitializationSrvMsg.h>
#include <suitbot_ros/SpeechSrv.h>
#include <suitbot_ros/ResetNode.h>

//#include "localization/initialization.h"


typedef enum state_num {
    MANUAL_MODE,
    IDLE,
    INITIALIZATION,
    GUIDING,
    ERROR
} state_t;


class JobManager
{
public:
    JobManager(ros::NodeHandle* nodehandle, parameters_t &_params); 
    
    parameters_t params;

    ros::NodeHandle nh; 
    ros::Subscriber audio_sub;
    ros::Subscriber odom_sub; 
    ros::Subscriber drive_state_sub; 

    ros::ServiceClient initialization_cli;

    ros::ServiceClient waypoint_cli; // reset course, point array

    ros::ServiceClient audio_cli; // enable or disable audio listening
    ros::ServiceClient speech_cli;

    ros::ServiceClient tracker_cli;
    ros::ServiceClient localization_cli;
    ros::ServiceClient planner_cli;

    state_t state = IDLE;

    int direction;

    // TODO A planner node

    double goal_x, goal_y;
    int global_state = 0;

    
    double val_from_subscriber_; 
    double val_to_remember_;
    
    void initializeSubscribers(); 
    void initializePublishers();
    void initializeServices();

    void get_goal_coordinate(int goal, double &goal_x, double &goal_y);


    void audio_cmd_subscriber_callback(const std_msgs::Int32 &msg_in); 
    void localization_callback(const nav_msgs::Odometry::ConstPtr& msg_in);
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

    void drive_state_callback(const std_msgs::Int8::ConstPtr& msg_in);

    int initialization_handler();
    int guiding_handler();
    int error_handler();

    int try_speak(std::string message);
    int set_mic_en_dis(bool state);

    bool last_cmd_halt = false;
}; 

#endif /* JOB_MANAGEMENT_H_ */
