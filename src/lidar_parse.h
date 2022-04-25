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


class LidarParse
{
public:
    LidarParse(ros::NodeHandle* nodehandle, parameters_t &_params); 
    
    parameters_t params;

    ros::NodeHandle nh; 
    
    ros::Subscriber pointcloud_sub;
    ros::Publisher local_map_pub;
    
    image_transport::ImageTransport it;
    image_transport::Publisher local_map_img_pub;
    
    LidarPointCloudPtr point_cloud_map;

    void initializeSubscribers(); 
    void initializePublishers();

    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg); 
}; 
