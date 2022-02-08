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

#include "initialization.h"


InitializationNode::InitializationNode(ros::NodeHandle* nodehandle):nh(*nodehandle) {
    ROS_INFO("Initializing Corner Extractor...");
    initializeSubscribers();
    initializePublishers();
    initializeServices();
    
    //initialize variables here, as needed
    val_to_remember_=0.0; 
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}


void InitializationNode::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh.subscribe("example_class_input_topic", 1, &InitializationNode::subscriberCallback,this);  
    // add more subscribers here, as needed
}


void InitializationNode::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh.advertiseService("example_minimal_service",
                                                   &InitializationNode::serviceCallback,
                                                   this);  
    // add more services here, as needed
}


void InitializationNode::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh.advertise<std_msgs::Float64>("example_class_output_topic", 1, true); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}



void InitializationNode::subscriberCallback(const std_msgs::Float64& message_holder) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of InitializationNode can access it
    ROS_INFO("myCallback activated: received value %f",val_from_subscriber_);
    std_msgs::Float64 output_msg;
    val_to_remember_ += val_from_subscriber_; //can use a member variable to store values between calls; add incoming value each callback
    output_msg.data= val_to_remember_;
    // demo use of publisher--since publisher object is a member function
    minimal_publisher_.publish(output_msg); 
}



bool InitializationNode::serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    ROS_INFO("service callback activated");
    response.success = true; // boring, but valid response info
    response.message = "here is a response string";
    return true;
}

int InitializationNode::run() {
    return 0;
}



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "initializationNode"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type InitializationNode");
    InitializationNode cornerExtractor(&nh);  //instantiate an InitializationNode object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 