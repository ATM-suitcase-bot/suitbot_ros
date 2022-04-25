#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <string>
#include <dirent.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "lidar.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/opencv.hpp> 

using namespace std;
using namespace Eigen;


int main(int argc, char **argv)
{
    struct dirent *d;
    DIR *dr;
    vector<string> names;

    dr = opendir("/home/tina/bag");
    if(dr!=NULL)
    {
        cout<<"List of Files & Folders:-\n";
        for(d=readdir(dr); d!=NULL; d=readdir(dr))
        {
            cout<<d->d_name<<endl;
            string s = d->d_name;
            if (s.find(".pcd") != std::string::npos)
                names.push_back(s);
        }
        closedir(dr);
    }
    else
        cout<<"\nError Occurred!";
    cout<<endl;

    cout << "num files: " << names.size() << endl;
    sort(names.begin(), names.end());

    LidarPointCloudPtr cloud(new LidarPointCloud);
    for (int i = 0; i < names.size() - 600; i+=5)
    {
        LidarPointCloudPtr cloud_tmp(new LidarPointCloud);
        string name = names[i];
        cout << "processing frame " << i << " / " << names.size() << endl;
        name = "/home/tina/bag/" + name;
        cout << "name: " << name << endl;
        if (pcl::io::loadPCDFile<LidarPoint>(name, *cloud_tmp) == -1)
        {
            cout << "Couldn't read pcd file." << endl;
            return -1;
        }
        cloud->points.insert(cloud->points.end(), cloud_tmp->points.begin(), cloud_tmp->points.end());
    }
    cloud->width = 1;
    cloud->height = cloud->points.size();
    pcl::io::savePCDFileASCII ("/home/tina/Documents/atm_ws/test_data/wean_map.pcd", *cloud);

    
    return 0;
}