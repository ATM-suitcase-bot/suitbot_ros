/**
 * @file job_management.cpp
 *
 * @brief Handles high-level job management of the robot
 *
 * @date 02/05/2022
 *
 * @author Tina Tian (yutian)
 */

#include "job_management.h"


JobManager::JobManager(ros::NodeHandle *nodehandle) : nh(*nodehandle)
{
    ROS_INFO("Initializing Corner Extractor...");
    initializeSubscribers();
    initializePublishers();
    initializeServices();

    // initialize variables here, as needed
    val_to_remember_ = 0.0;

    // can also do tests/waits to make sure all required services, topics, etc are alive
}

void JobManager::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    audio_sub = nh.subscribe("audio_topic", 1, &JobManager::audio_cmd_subscriber_callback, this);
    odom_sub = nh.subscribe("odom_topic", 1, &JobManager::localization_callback, this);
    // add more subscribers here, as needed
}

void JobManager::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh.advertiseService("example_minimal_service",
                                            &JobManager::serviceCallback,
                                            this);
    // add more services here, as needed
    initialization_cli = nh.serviceClient<suitbot_ros::InitializationSrvMsg>("init_pose");
}

void JobManager::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh.advertise<std_msgs::Float64>("example_class_output_topic", 1, true);
    // add more publishers, as needed
    //  note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}



void JobManager::get_goal_coordinate(int goal, double &goal_x, double &goal_y) {
    goal_x = 0;
    goal_y = 0;
}


void JobManager::audio_cmd_subscriber_callback(const std_msgs::Int32 &msg_in)
{
    // if it's in idle state, go to initialization state
    if (state == IDLE)
    {
        
        if (msg_in.data != CANCEL_JOB)
        {
            //get_goal_coordinate(msg_in.goal, goal_x, goal_y);
            state = INITIALIZATION;
        }
    }
    // elif it's in INITIALIZATION or GUIDING state, possibly go to complete state
    else
    {
        if (msg_in.data == CANCEL_JOB)
        {
            // cancel current job

            state = IDLE;
        }
    }
    //
}

void JobManager::localization_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
{
    // subscribe to odometry during guiding phase

}

bool JobManager::serviceCallback(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response)
{
    ROS_INFO("service callback activated");
    response.success = true; // boring, but valid response info
    response.message = "here is a response string";
    return true;
}


int JobManager::initialization_handler()
{
    suitbot_ros::InitializationSrvMsg srv_msg;
    srv_msg.request.header.stamp = ros::Time::now();
    srv_msg.request.wait_time = 10;
    if (initialization_cli.call(srv_msg))
    {
        ROS_INFO("Successfully initialized starting pose!");
    }
    else {
        // fail to initialize robot pose, cancel current job
        ROS_INFO("Failed to initialize starting pose, going back to IDLE...");
        state = IDLE;
    }
    return 0;
}

int JobManager::guiding_handler()
{
    return 0;
}


int JobManager::error_handler()
{
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jobManager");
    ros::NodeHandle nh;

    ROS_INFO("main: instantiating an object of type JobManager");
    JobManager jobManager(&nh);


    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}
