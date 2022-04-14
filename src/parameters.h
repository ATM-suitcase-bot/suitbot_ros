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

    bool manual_control;
    bool use_audio;

    int course_idx;

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

    string LIDAR_SYNC_TOPIC;


    // for localization
    // string base_frame_id;   /*!< Name of the robot TF */
    // string odom_frame_id;   /*!< Name of the flight origin of the robot TF */
    // string global_frame_id; /*!< Name of the test-bed origin TF */
    // string map_path;        /*!< Route to the localization of the environment map  */

    // bool set_initial_pose; /*!< Flag to indicate if t he initial pose has been received */

    // double init_x; /*!< Start x-axis position */
    // double init_y; /*!< Start y-axis position */
    // double init_z; /*!< Start z-axis position */
    // double init_a; /*!< Start yaw angle */

    // double init_x_dev; /*!< Thresholds x-axis position in initialization*/
    // double init_y_dev; /*!< Thresholds y-axis position in initialization*/
    // double init_z_dev; /*!< Thresholds z-axis position in initialization*/
    // double init_a_dev; /*!< Thresholds yaw angle in initialization*/

    // double grid_slice_z;             /*!< Height of grid slice */
    // double publish_point_cloud_rate; /*!< Map point cloud publishing rate */
    // double publish_grid_slice_rate;  /*!< map grid slice publishing rate */

    // double sensor_dev;   /*!< Desviation of 3D point cloud sensor */
    // double sensor_range; /*!< Desviation of measurement of radio-range sensor */
    // double voxel_size;   /*!< Size of voxel grid filter */

    // int num_particles; /*!< Particle number in the filter */

    // double odom_x_mod; /*!< Thresholds x-axis position in the prediction */
    // double odom_y_mod; /*!< Thresholds y-axis position in the prediction */
    // double odom_z_mod; /*!< Thresholds z-axis position in the prediction */
    // double odom_a_mod; /*!< Thresholds yaw angle in the prediction */

    // int resample_interval; /*!< Resampling control */

    // double update_rate; /*!< Filter updating frequency */
    // double d_th;        /*!< Threshold in the distance for the update */
    // double a_th;        /*!< Threshold in yaw angle for the update */



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

        LIDAR_SYNC_TOPIC = readParam<string>(n, "lidar_sync_topic");

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

        manual_control = readParam<bool>(n, "manual_control");

        use_audio = readParam<bool>(n, "use_audio");

        course_idx = readParam<int>(n, "course_idx");
    }

} parameters_t;



#endif /* PARAMETERS_H_ */