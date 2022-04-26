#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../lidar.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/opencv.hpp> 

#include "../occupancy_map.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    // unit test functions
    vector<vector<int>> arr = {{0,0,0,0,0,0,0,0,0},
                               {0,FREE,0,FREE,FREE,FREE,FREE,FREE,0},
                               {0,0,0,FREE,FREE,FREE,FREE,FREE,0},
                               {0,FREE,FREE,FREE,FREE,FREE,0,0,0},
                               {0,FREE,FREE,FREE,0,0,0,FREE,0},
                               {0,0,0,0,0,0,0,0,0}};
    OccupancyMap occ;
    occ.initOccupancyGridMap(arr);

    for (int i = 0; i < 6; i++) 
    {
        for (int j = 0; j < 9; j++)
            std::cout << occ.occupancy_map[i][j] << "\t";
        std::cout << endl;
    }
    return 0;
}