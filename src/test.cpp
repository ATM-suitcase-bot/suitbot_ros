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

using namespace std;
using namespace Eigen;


int main(int argc, char **argv)
{
    // unit test functions
    vector<vector<int>> arr = {{0,0,0,0,0,0,0,0,0},
                               {0,1,0,1,1,1,1,1,0},
                               {0,0,0,0,1,0,0,1,0},
                               {0,1,0,1,1,1,0,0,0},
                               {0,1,1,1,0,0,0,1,0},
                               {0,0,0,0,0,0,0,0,0}};
    floodFill(arr, 6, 9, 1, 4, 0, 2);
    for (int i = 0; i < 6; i++) 
    {
        for (int j = 0; j < 9; j++)
            std::cout << arr[i][j];
        std::cout << endl;
    }
    return 0;
}
