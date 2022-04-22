/**
 * @file lidar_feature_extraction.h
 *
 * @brief Adapted from SSL_SLAM2
 *
 * @date 01/18/2022
 *
 * @author Tina Tian (yutian)
 */

#include "lidar.h"

lidar::Lidar::Lidar(){
 
}


void lidar::Lidar::setLines(double num_lines_in){
    num_lines=num_lines_in;
}


void lidar::Lidar::setVerticalAngle(double vertical_angle_in){
    vertical_angle = vertical_angle_in;
}


void lidar::Lidar::setVerticalResolution(double vertical_angle_resolution_in){
    vertical_angle_resolution = vertical_angle_resolution_in;
}


void lidar::Lidar::setScanPeriod(double scan_period_in){
    scan_period = scan_period_in;
}


void lidar::Lidar::setMaxDistance(double max_distance_in){
	max_distance = max_distance_in;
}

void lidar::Lidar::setMinDistance(double min_distance_in){
	min_distance = min_distance_in;
}

Eigen::Vector3f lidar::pcl2eigen(LidarPoint pt){
    Eigen::Vector3f res(pt.x, pt.y, pt.z);
    return res;
}