/**
 * @file parameters.h
 *
 * @brief Define parameters
 *
 * @date 02/05/2021
 *
 * @author Tina Tian (yutian)  
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <string>
#include <ros/ros.h>
#include <XmlRpcValue.h>
using namespace std;

extern bool parameter_set;

extern string map_file;

// command types
extern int CANCEL_JOB;
extern int LEFT;
extern int MIDDLE;
extern int RIGHT;
extern int NOTHING;

// topics
extern string SPEECH_SERVICE;
extern string LISTENING_SERVICE;
extern string USR_CMD_TOPIC;
extern string CTRL_TOPIC;
extern string FORCE_TOPIC;
extern string ENCODER_TOPIC;
extern string RESET_PATH_SERVICE;
extern string PLANNER_ARROW_TOPIC;
extern string GLOBAL_MAP_TOPIC;
extern string PLANNED_PATH_TOPIC;


void readParameters(ros::NodeHandle &n);


#endif /* PARAMETERS_H_ */