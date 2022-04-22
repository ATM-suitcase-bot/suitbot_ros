/**
 * @file localization_node.h
 *
 * @brief Use a PF for localization
 *
 * @date 02/05/2022
 *
 * @author Tina Tian (yutian)  
 */

#ifndef LOCALIZATION_NODE_H_
#define LOCALIZATION_NODE_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> 

#include <std_msgs/Bool.h> 
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h> 

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Dense>
#include "../parameters.h"
#include "../lidar.h"

#include "particle_filter.h"


class LocalizationNode
{
public:
    LocalizationNode(ros::NodeHandle* nodehandle, parameters_t &_params); 

    int run();
    

    ros::NodeHandle nh; 
    ros::Subscriber point_sub;
    ros::Subscriber odom_sub;  
    ros::Publisher particles_pose_pub;
    
    parameters_t params;

    ParticleFilter pf;


    Particle mean_p;              /*!< Instance of the Particle struct for particles of filter */
    Particle lastmean_p;          /*!< Instance of the Particle struct for previous update particles of filter */

    ros::Time nextupdate_time; /*!< Timer for the next update  */
    //ros::Publisher cloud_filter_pub;   /*!< Filtered point cloud publisher */

    tf::Transform base_2_odom_tf;              /*!< Base-odom Transformation */

    Eigen::Vector3f prev_odom;
    Eigen::Vector3f cur_odom;

    // 2D map, discretized with resolution of 50cm
    

    // 3D point cloud map

    bool is_odom{ false };  /*!< Flag to know the initialize of odometry */
    
    void initializeSubscribers(); 
    void initializePublishers();
    void initializeServices();
    
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    void odomCallback(const nav_msgs::Odometry &ctrl_in);

    void publishParticles();

    bool checkUpdateThresholds();

    void setInitialPose(const tf::Transform& init_pose, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev);

    double getYawFromTf(const tf::Transform& tf);

}; 

#endif /* LOCALIZATION_NODE_H_ */