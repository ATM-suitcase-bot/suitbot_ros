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
#include <Eigen/Dense>

using namespace std;


template<typename T>
T readParam(ros::NodeHandle &n, string name)
{
    T ans;
    if (!n.getParam(name, ans))
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}


typedef struct params
{

    string map_file;
    string pcd_file;

    bool manual_control;
    bool use_audio;

    int course_idx;

    // command types
    int CANCEL_JOB;
    int LEFT;
    int MIDDLE;
    int RIGHT;
    int NOTHING;
    int ELEV;
    int SIX;
    int NINE;
    int FOUR;
    int FOUNT;
    int STAIR;

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

    string LOCAL_MAP_TOPIC;

    string LOCAL_MAP_IMAGE_TOPIC;

    string PCD_MAP_TOPIC;

    // for localization
    // string base_frame_id;   /*!< Name of the robot TF */
    // string odom_frame_id;   /*!< Name of the flight origin of the robot TF */
    string global_frame_id; /*!< Name of the test-bed origin TF */
    string PF_PARTICLES_TOPIC;

    // bool set_initial_pose; /*!< Flag to indicate if t he initial pose has been received */

    // double init_x; /*!< Start x-axis position */
    // double init_y; /*!< Start y-axis position */
    // double init_z; /*!< Start z-axis position */
    // double init_a; /*!< Start yaw angle */

    float pf_alpha1, pf_alpha2, pf_alpha3, pf_alpha4;
    float pf_sigma_hit, pf_lambda_short, pf_max_range, pf_max_span;
	float pf_z_hit, pf_z_short, pf_z_max, pf_z_rand;

    // double grid_slice_z;             /*!< Height of grid slice */
    double publish_point_cloud_rate;    /*!< Map point cloud publishing rate */
    // double publish_grid_slice_rate;  /*!< map grid slice publishing rate */

    // double sensor_dev;   /*!< Desviation of 3D point cloud sensor */
    // double sensor_range; /*!< Desviation of measurement of radio-range sensor */
    double voxel_size;   /*!< Size of voxel grid filter */

    // int num_particles; /*!< Particle number in the filter */

    // double odom_x_mod; /*!< Thresholds x-axis position in the prediction */
    // double odom_y_mod; /*!< Thresholds y-axis position in the prediction */
    // double odom_z_mod; /*!< Thresholds z-axis position in the prediction */
    // double odom_a_mod; /*!< Thresholds yaw angle in the prediction */

    int pf_resample_interval; /*!< Resampling control */

    double pf_update_rate; /*!< Filter updating frequency */
    float pf_update_dist_threshold;        /*!< Threshold in the distance for the update */
    float pf_update_angle_threshold;        /*!< Threshold in yaw angle for the update */

    int pf_init_num_particles_per_grid;
    int pf_init_num_particles_total;

    float fixed_height;

    Eigen::Affine3f pcd_map_transformation;
    

    double local_map_xmin;
    double local_map_xmax;
    double local_map_ymin;
    double local_map_ymax;
    double local_map_zmin;
    double local_map_zmax;

    float local_icp_dt_thresh;
    float local_icp_dq_thresh;

    float local_map_lookahead;

    float local_map_resolution;
    float global_map_resolution;

    float obstacle_zmin;
    float obstacle_zmax;
    
    XmlRpc::XmlRpcValue state_map;

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

        LOCAL_MAP_TOPIC = readParam<string>(n, "local_map_topic");
        LOCAL_MAP_IMAGE_TOPIC = readParam<string>(n, "local_map_image_topic");

        PCD_MAP_TOPIC = readParam<string>(n, "pcd_map_topic");
        PF_PARTICLES_TOPIC = readParam<string>(n, "pf_particles_topic");

        // command types
        state_map = readParam<XmlRpc::XmlRpcValue>(n, "states_map");
        //cout << (state_map);
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
            else if (name == "elevator")
                ELEV = value;
            else if (name == "nineteen")
                NINE = value;
            else if (name == "four")
                FOUR = value;
            else if (name == "six")
                SIX = value;
            else if (name == "fountain")
                FOUNT = value;
            else if (name == "staircase")
                STAIR = value;
            else if (name == "nothing")
                NOTHING = value;
            else
                ROS_ERROR("Parameter %s in states_map is not needed", name);
        } 

        map_file = readParam<string>(n, "map_file");
        pcd_file = readParam<string>(n, "pcd_file");

        manual_control = readParam<bool>(n, "manual_control");

        use_audio = readParam<bool>(n, "use_audio");

        course_idx = readParam<int>(n, "course_idx");

        pf_update_rate = readParam<double>(n, "pf_update_rate");
        voxel_size = readParam<float>(n, "voxel_size");
        global_frame_id = readParam<string>(n, "global_frame_id");

        fixed_height = readParam<float>(n, "fixed_height");

        pf_resample_interval = readParam<int>(n, "pf_resample_interval");
        pf_update_dist_threshold = readParam<float>(n, "pf_update_dist_threshold");
        pf_update_angle_threshold = readParam<float>(n, "pf_update_angle_threshold");

        pf_init_num_particles_per_grid = readParam<int>(n, "pf_init_num_particles_per_grid");
        pf_init_num_particles_total = readParam<int>(n, "pf_init_num_particles_total");

        publish_point_cloud_rate = readParam<double>(n, "publish_point_cloud_rate");

        pf_alpha1 = readParam<float>(n, "pf_alpha1");
        pf_alpha2 = readParam<float>(n, "pf_alpha2");
        pf_alpha3 = readParam<float>(n, "pf_alpha3");
        pf_alpha4 = readParam<float>(n, "pf_alpha4");

        pf_sigma_hit = readParam<float>(n, "pf_sigma_hit");
        pf_lambda_short = readParam<float>(n, "pf_lambda_short");
        pf_max_range = readParam<float>(n, "pf_max_range");
        pf_max_span = readParam<float>(n, "pf_max_span");
        pf_z_hit = readParam<float>(n, "pf_z_hit");
        pf_z_short = readParam<float>(n, "pf_z_short");
        pf_z_max = readParam<float>(n, "pf_z_max");
        pf_z_rand = readParam<float>(n, "pf_z_rand");


        // pcd map transformation stuff
        float tx, ty, tz, qx, qy, qz, qw;
        tx = readParam<float>(n, "tx");
        ty = readParam<float>(n, "ty");
        tz = readParam<float>(n, "tz");
        qx = readParam<float>(n, "qx");
        qy = readParam<float>(n, "qy");
        qz = readParam<float>(n, "qz");
        qw = readParam<float>(n, "qw");

        pcd_map_transformation = Eigen::Affine3f::Identity();
        pcd_map_transformation.translation() << tx, ty, tz;
        Eigen::Quaternionf q;
        q.x() = qx;
        q.y() = qy;
        q.z() = qz;
        q.w() = qw;
        pcd_map_transformation.rotate(q);
        
        // local map stuff
        local_map_xmin = readParam<double>(n, "local_map_xmin");
        local_map_xmax = readParam<double>(n, "local_map_xmax");
        local_map_ymin = readParam<double>(n, "local_map_ymin");
        local_map_ymax = readParam<double>(n, "local_map_ymax");
        local_map_zmin = readParam<double>(n, "local_map_zmin");
        local_map_zmax = readParam<double>(n, "local_map_zmax");

        local_icp_dt_thresh = readParam<float>(n, "local_icp_dt_thresh");
        local_icp_dq_thresh = readParam<float>(n, "local_icp_dq_thresh");

        local_map_lookahead = readParam<float>(n, "local_map_lookahead");

        local_map_resolution = readParam<float>(n, "local_map_resolution");
        global_map_resolution = readParam<float>(n, "global_map_resolution");

        obstacle_zmin = readParam<float>(n, "obstacle_zmin");
        obstacle_zmax = readParam<float>(n, "obstacle_zmax");
    }

} parameters_t;



#endif /* PARAMETERS_H_ */
