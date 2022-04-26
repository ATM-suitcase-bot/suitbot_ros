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


JobManager::JobManager(ros::NodeHandle *nodehandle, parameters_t &_params) : nh(*nodehandle), params(_params)
{
    ROS_INFO("Initializing Job Manager...");
    initializeSubscribers();
    initializePublishers();
    initializeServices();

    
    direction = params.NOTHING;
}

void JobManager::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    audio_sub = nh.subscribe(params.USR_CMD_TOPIC, 1, &JobManager::audio_cmd_subscriber_callback, this);
    odom_sub = nh.subscribe("odom_topic", 1, &JobManager::localization_callback, this);
    // add more subscribers here, as needed
}

void JobManager::initializeServices()
{
    ROS_INFO("Initializing Services");
    //minimal_service_ = nh.advertiseService("example_minimal_service", &JobManager::serviceCallback, this);
    // add more services here, as needed
    //initialization_cli = nh.serviceClient<suitbot_ros::InitializationSrvMsg>("init_pose");
    audio_cli = nh.serviceClient<std_srvs::SetBool>(params.LISTENING_SERVICE);
    speech_cli = nh.serviceClient<suitbot_ros::SpeechSrv>(params.SPEECH_SERVICE);
}

void JobManager::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    //minimal_publisher_ = nh.advertise<std_msgs::Float64>("example_class_output_topic", 1, true);
    // add more publishers, as needed
    //  note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}



void JobManager::get_goal_coordinate(int goal, double &goal_x, double &goal_y) {
    goal_x = 0;
    goal_y = 0;
}


void JobManager::audio_cmd_subscriber_callback(const std_msgs::Int32 &msg_in)
{
    if (state == MANUAL_MODE)
        return;
    // if it's in idle state, go to initialization state
    if (state == IDLE)
    {
        
        if (msg_in.data != params.CANCEL_JOB && msg_in.data != params.NOTHING)
        {
            //get_goal_coordinate(msg_in.goal, goal_x, goal_y);
            state = GUIDING;
            direction = msg_in.data;
            // start path planning and following job. we use state to control run or not run

        }
    }
    // elif it's in INITIALIZATION or GUIDING state, possibly go to complete state
    else
    {
        if (msg_in.data == params.CANCEL_JOB)
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
    /*
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
    */
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
    ros::init(argc, argv, "job_management");
    ros::NodeHandle nh;

    parameters_t params;
    params.readParameters(nh);

    ROS_INFO_STREAM("main: instantiating an object of type JobManager");
    JobManager jobManager(&nh, params);


    bool mic_enabled = false;

    ros::Rate loop_rate(1);
    int counter = 0;
    int counter_state = 0;
    // wait for 5 seconds til all nodes are up
    sleep(4);

    if (params.manual_control == false && params.use_audio == true) {
        // say something
        suitbot_ros::SpeechSrv speech;
        speech.request.data = "Robot initialized, where do you want to go?";
        if (jobManager.speech_cli.call(speech)){
            ROS_INFO("Spoken successfully: %s", speech.request.data.c_str());
        }
        else {
            ROS_INFO("fail to speak");
        }
        // enable audio listening
        std_srvs::SetBool srv_mic;
        srv_mic.request.data = true;
        if (jobManager.audio_cli.call(srv_mic))
        {
            ROS_INFO("Audio listener enabled");
            mic_enabled = true;
        }
        else {
            ROS_INFO("fail to enable listening");
        }
    }
    else if (params.manual_control == true) { // manual control, we set the state to MANUAL. no audio
        jobManager.state = MANUAL_MODE;
    }

    while (ros::ok())
    {
        
        if (jobManager.state == GUIDING && counter_state == 0)
        {
            std::cout << "guiding! direction: " << int(jobManager.direction) << std::endl;
            std::string dir;
            if (jobManager.direction == params.LEFT)
                dir = "left";
            else if (jobManager.direction == params.MIDDLE)
                dir = "middle";
            else if (jobManager.direction == params.RIGHT)
                dir = "right";
		else
		    dir = "middle"; //Set middle as the default, while testing out other audio options

            if (params.use_audio)
            {
                suitbot_ros::SpeechSrv speech;
                speech.request.data = "Received command. Going " + dir;
                if (jobManager.speech_cli.call(speech)){
                    ROS_INFO("Spoken successfully: %s", speech.request.data.c_str());
                }
                else {
                    ROS_INFO("fail to speak");
                }
            }
            counter_state += 1;
            if (params.use_audio)
            {
                std_srvs::SetBool srv_mic;
                srv_mic.request.data = false;
                if (jobManager.audio_cli.call(srv_mic))
                {
                    ROS_INFO("Audio listener disabled");
                    mic_enabled = true;
                }
                else {
                    ROS_INFO("fail to enable listening");
                }
            }
        }
        ros::spinOnce();

        loop_rate.sleep();
    }
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}
