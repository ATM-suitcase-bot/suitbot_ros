/**
 * @file lidar_feature_extraction.h
 *
 * @brief Adapted from SSL_SLAM2
 *
 * @date 01/18/2022
 *
 * @author Tina Tian (yutian)
 */

#ifndef _LIDAR_H_
#define _LIDAR_H_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//define lidar parameter

typedef pcl::PointXYZ LidarPoint;
typedef pcl::PointCloud<LidarPoint> LidarPointCloud;
typedef pcl::PointCloud<LidarPoint>::Ptr LidarPointCloudPtr;
typedef pcl::PointCloud<LidarPoint>::ConstPtr LidarPointCloudConstPtr;

namespace lidar{

class Lidar
{
    public:
        Lidar();

        void setScanPeriod(double scan_period_in);
        void setLines(double num_lines_in);
        void setVerticalAngle(double vertical_angle_in);
        void setVerticalResolution(double vertical_angle_resolution_in);
        //by default is 100. pls do not change
        void setMaxDistance(double max_distance_in);
        void setMinDistance(double min_distance_in);

    	double max_distance;
        double min_distance;
        int num_lines;
        double scan_period;
        int points_per_line;
        double horizontal_angle_resolution;
        double horizontal_angle;
        double vertical_angle_resolution;
        double vertical_angle;
};

Eigen::Vector3f pcl2eigen(LidarPoint pt);

}

#endif // _LIDAR_H_

