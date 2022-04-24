#ifndef LOCAL_MAP_GENERATION_H_
#define LOCAL_MAP_GENERATION_H_

#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <vector>

#include <ros/ros.h> 

#include "parameters.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "lidar.h"
#include "utility/pcl_utils.h"
#include "utility/utils.h"
#include "occupancy_map.h"

#include <suitbot_ros/LocalMapMsg.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class LocalMapGenerator
{
public:
    LocalMapGenerator(ros::NodeHandle* nodehandle, parameters_t &_params); 
    
    parameters_t params;

    ros::NodeHandle nh; 
    
    ros::Subscriber odom_sub; 
    ros::Subscriber pointcloud_sub;
    
    ros::Publisher local_map_pub;

    image_transport::ImageTransport it;
    image_transport::Publisher local_map_img_pub;

    int direction;

    // convention: z = 0 is floor, +z is towards the ceiling
    LidarPointCloudPtr point_cloud_map;

    double goal_x, goal_y;

    Eigen::Vector3f cur_trans_w;
    Eigen::Quaternionf cur_rot_w;


    void initializeSubscribers(); 
    void initializePublishers();
    void initializeServices();

    void get_goal_coordinate(int goal, double &goal_x, double &goal_y);

    void odom_callback(const nav_msgs::Odometry &ctrl_in);

    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg); 
}; 

#endif /* LOCAL_MAP_GENERATION_H_ */