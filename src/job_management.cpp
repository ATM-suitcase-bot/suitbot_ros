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
    drive_state_sub = nh.subscribe(params.DRIVE_STATUS_TOPIC, 1, &JobManager::drive_state_callback, this);
}

void JobManager::initializeServices()
{
    ROS_INFO("Initializing Services");
    //minimal_service_ = nh.advertiseService("example_minimal_service", &JobManager::serviceCallback, this);
    // add more services here, as needed
    //initialization_cli = nh.serviceClient<suitbot_ros::InitializationSrvMsg>("init_pose");
    audio_cli = nh.serviceClient<std_srvs::SetBool>(params.LISTENING_SERVICE);
    speech_cli = nh.serviceClient<suitbot_ros::SpeechSrv>(params.SPEECH_SERVICE);
    tracker_cli = nh.serviceClient<suitbot_ros::ResetNode>(params.RESET_TRACKER_SERVICE);
    
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

void JobManager::drive_state_callback(const std_msgs::Int8::ConstPtr& msg_in){

    std::cout << "job state callback: " << std::to_string((msg_in->data)) << "\n";
    if(msg_in->data == 1){
        last_cmd_halt = false;
    }
    else if(msg_in->data == 2 and state == GUIDING){
        state = IDLE;

        //--- reset all nodes ---

        //Reset tracker
        suitbot_ros::ResetNode reset_msg;
        reset_msg.request.data = true;
        this->tracker_cli.call(reset_msg);

        //--- reset job man params ---
        
    }
    else if(msg_in->data == 3 and !last_cmd_halt){
        try_speak("Unavoidable or excessively close obstacles detected. Halting.")
        last_cmd_halt = true;
    }
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

//Attempts to send an audio output, returns 1 on success and 0 on failure
int JobManager::try_speak(std::string message)
{
    suitbot_ros::SpeechSrv speech;
    speech.request.data = message;
    if (this->speech_cli.call(speech)){
        ROS_INFO("Spoken successfully: %s", speech.request.data.c_str());
        return 1;
    }
    else {
        ROS_INFO("fail to speak");
        return 0;
    }
}

int JobManager::set_mic_en_dis(bool state)
{
    std_srvs::SetBool srv_mic;
    srv_mic.request.data = state;
    if (audio_cli.call(srv_mic))
    {
        ROS_INFO("Audio listener state set");
        return state;
    }
    else {
        ROS_INFO("Failed to set audio listener state");
        return false;
    }
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
    sleep(4); //CHANGE TO WAIT FOR SYSTEM SPIN

    while (ros::ok())
    {
        //OVERRIDING manual state will disable other modes
        if (params.manual_control == true) { // manual control, we set the state to MANUAL. no audio
            jobManager.state = MANUAL_MODE;
        }


        if (jobManager.state = IDLE && params.use_audio == true) {
            // say something
            jobManager.try_speak("Robot initialized, where do you want to go?");
        
        
            // enable audio listening
            mic_enabled = jobManager.set_mic_en_dis(true);
            
        }
        

        if (jobManager.state == GUIDING && counter_state == 0)
        {
            std::cout << "guiding! direction: " << int(jobManager.direction) << " " << params.ELEV << std::endl;
            std::string dir;
            if (jobManager.direction == params.LEFT)
                dir = "left";
            else if (jobManager.direction == params.MIDDLE)
                dir = "middle";
            else if (jobManager.direction == params.RIGHT)
                dir = "right";
            else if (jobManager.direction == params.ELEV)
                dir = "elevator";
            else if (jobManager.direction == params.NINE)
                dir = "seven three nineteen";
            else if (jobManager.direction == params.FOUR)
                dir = "seven four two four";
            else if (jobManager.direction == params.SIX)
                dir = "classroom six";
            else if (jobManager.direction == params.FOUNT)
                dir = "water fountain";
            else if (jobManager.direction == params.STAIR)
                dir = "doherty staircase";
            else
                dir = "error"; //this will give an audible error if very confused

            if (params.use_audio)
            {
                jobManager.try_speak("Received command. Going to " + dir);
            }
            counter_state += 1;
            mic_enabled = jobManager.set_mic_en_dis(false);
        }
        ros::spinOnce();

        loop_rate.sleep();
    }
    ROS_INFO("Job management main loop exited");
    ros::spin();
    return 0;
}
