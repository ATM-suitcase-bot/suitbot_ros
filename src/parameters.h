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


template<typename T>
T readParam(ros::NodeHandle &n, string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    } else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}


typedef struct params
{

    string map_file;

    // command types
    int CANCEL_JOB;
    int LEFT;
    int MIDDLE;
    int RIGHT;
    int NOTHING;

    // topics
    string SPEECH_SERVICE;
    string LISTENING_SERVICE;
    string USR_CMD_TOPIC;
    string CTRL_TOPIC;
    string FORCE_TOPIC;
    string ENCODER_TOPIC;
    string RESET_PATH_SERVICE;
    string PLANNER_ARROW_TOPIC;
    string GLOBAL_MAP_TOPIC;
    string PLANNED_PATH_TOPIC;


    void readParameters(ros::NodeHandle &n)
    {
        // topics
        SPEECH_SERVICE = readParam<string>(n, "speech_service");
        LISTENING_SERVICE = readParam<string>(n, "listening_service");
        USR_CMD_TOPIC = readParam<string>(n, "usr_cmd_topic");

        CTRL_TOPIC = readParam<string>(n, "ctrl_topic");
        FORCE_TOPIC = readParam<string>(n, "force_topic");
        ENCODER_TOPIC = readParam<string>(n, "encoder_topic");

        RESET_PATH_SERVICE = readParam<string>(n, "reset_path_service");
        PLANNER_ARROW_TOPIC = readParam<string>(n, "planner_arrow_topic");
        GLOBAL_MAP_TOPIC = readParam<string>(n, "global_map_topic");
        PLANNED_PATH_TOPIC = readParam<string>(n, "planned_path_topic");

        // command types
        auto state_map = readParam<XmlRpc::XmlRpcValue>(n, "states_map");
        for (int i = 0; i < state_map.size(); i++) {
            auto sublist = state_map[i];
            std::stringstream ss;
            ss << sublist["name"];
            string name = ss.str();
            int value = sublist["key"];
            ROS_INFO("Parameter name: %s, value, %d", name.c_str(), value);
            if (name == "cancel")
                CANCEL_JOB = value;
            else if (name == "left")
                LEFT = value;
            else if (name == "middle")
                MIDDLE = value;
            else if (name == "right")
                RIGHT = value;
            else if (name == "nothing")
                NOTHING = value;
            else
                ROS_ERROR("Parameter %s in states_map is not needed", name);
        } 

        map_file = readParam<string>(n, "map_file");
    }

} parameters_t;



#endif /* PARAMETERS_H_ */