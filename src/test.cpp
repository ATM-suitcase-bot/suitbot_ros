#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include<dirent.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "lidar.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/opencv.hpp> 
#include "utility/utils.h"

#include <random>

using namespace std;
using namespace Eigen;


int main(int argc, char **argv)
{
    LidarPointCloudPtr cloud(new LidarPointCloud);
    string name = "/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good.pcd";
    if (pcl::io::loadPCDFile<LidarPoint>(name, *cloud) == -1)
    {
        cout << "Couldn't read pcd file." << endl;
        return -1;
    }
    // fill in hole
    float a, b, c, d;
    Vector3f A(14.573961, 6.168147, 0.349627);
    Vector3f B(14.796391, 5.222781, 0.354955);
    Vector3f C(15.048866, 3.578148, -0.526846);

    three_point_plane(A, B, C, a, b, c, d);

    int num_sample = 1000;
    std::random_device rd;
    std::uniform_real_distribution<float> dist_z(-0.8, 2.3);
    std::uniform_real_distribution<float> dist_y(4.5, 6.5);
    for (int i = 0; i < num_sample; i++)
    {
        float x = 0;
        float y = dist_y(rd);
        float z = dist_z(rd);
        x = (-d - b*y - c*z) / a;
        LidarPoint pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 
    }

    cloud->width = 1;
    cloud->height = cloud->points.size();
    pcl::io::savePCDFileASCII ("/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good_filled.pcd", *cloud);

    return 0;
}
