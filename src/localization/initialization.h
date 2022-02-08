/**
 * @file initialization.h
 *
 * @brief Responsible for coordinating localization, path planning, and mobility during initialization phase
 *
 * @date 02/05/2021
 *
 * @author Tina Tian (yutian)  
 */

#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 

#include <std_msgs/Bool.h> 
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h> 


class InitializationNode
{
public:
    InitializationNode(ros::NodeHandle* nodehandle); 

    int run();
    
private:
    ros::NodeHandle nh; 
    ros::Subscriber minimal_subscriber_; 
    ros::ServiceServer minimal_service_;
    ros::Publisher  minimal_publisher_;
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    double val_to_remember_; // member variables will retain their values even as callbacks come and go
    
    void initializeSubscribers(); 
    void initializePublishers();
    void initializeServices();
    
    void subscriberCallback(const std_msgs::Float64& message_holder); 
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

}; 

#endif /* INITIALIZATION_H_ */