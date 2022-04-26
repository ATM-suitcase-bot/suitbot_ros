#include <Eigen/Dense>
#include <iostream>
#include <cmath>

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
#include "utility/pcl_utils.h"
#include "occupancy_map.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    exit(0);
    // floodfill the image
    OccupancyMap occ;
    string map_name = "/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good_grid_edited.png";
    occ.initOccupancyGridMap(map_name, 0.25);
    floodFill(occ.occupancy_map, occ.rows, occ.cols, 207, 165, OCCUPIED, FREE);
    cv::Mat img;
    array_to_image(occ.occupancy_map, img);
    cv::imwrite("/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good_grid_edited_filled.png", img);
    cv::imshow("Gray scale", img); 
    cv::waitKey(0);


    exit(0);
    LidarPointCloudPtr cloud(new LidarPointCloud);

    LidarPointCloudPtr cloud_sliced(new LidarPointCloud);

    string filename_1 = "/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good_processed.pcd";
    string filename_2 = "/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good_processed_flat.pcd";

    if (pcl::io::loadPCDFile<LidarPoint>(filename_1, *cloud) == -1)
    {
        cout << "Couldn't read pcd file." << endl;
        return -1;
    }

    float a1, b1, c1, d1;
    Vector3f A1(-6.715748, 11.383018, -0.186463);
    Vector3f B1(-9.523818, 70.119072, 0.557460);
    Vector3f C1(15.463889, 69.861610, 0.395925);
    three_point_plane(A1, B1, C1, a1, b1, c1, d1);
    cout << "a1: " << a1 << ", b1: " << b1 << ", c1: " << c1 << endl;

    // check distance to plane and filter
    for (int i = 0; i < cloud->points.size(); i++)
    {
        float dist = abs(a1*cloud->points[i].x + b1*cloud->points[i].y + c1*cloud->points[i].z + d1) / sqrt(a1*a1 + b1*b1 + c1*c1);
        if (dist < 0.2)
            cloud_sliced->points.push_back(cloud->points[i]);
    }
    cloud_sliced->width = cloud_sliced->points.size();
    cloud_sliced->height = 1;


    // export to occ map
    LidarPoint minPt, maxPt;
    pcl::getMinMax3D (*cloud_sliced, minPt, maxPt);
    cout << "min x y z: "<< minPt.x << ",  " << minPt.y<< ", " <<minPt.z << endl;
    cout << "max x y z: "<< maxPt.x << ",  " << maxPt.y<< ", " <<maxPt.z << endl;

    pcl::io::savePCDFileASCII (filename_2, *cloud_sliced);


    // x in  -40 m to 40 m (horizontal), y in -40 to 80 (verticle), z 0
    cv::Mat outimg = cv::Mat::ones(320, 480, CV_8UC1) * 255;
    for (int i = 0; i < cloud_sliced->points.size(); i++)
    {
        float x = cloud_sliced->points[i].x;
        float y = cloud_sliced->points[i].y;
        if (x < 40.0 && x > -40.0 && y >-40.0 && y < 80.0) {
            int x_coord = int(floor((x + 40) / 0.25));
            int y_coord = int(floor((y + 40) / 0.25));
            //cout << "good" << endl;
            //cout << x_coord << ", " << y_coord << endl;
            outimg.at<uchar>(x_coord, y_coord) = 0;
        }
    }
    cv::imwrite("/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good_grid.png", outimg);
    cv::imshow("Gray scale", outimg); 
    cv::waitKey(0);

    // visualize
     pcl::visualization::PCLVisualizer viewer2("my_vis");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_color_handler(cloud_sliced, 80, 70, 242);
    // We add the point cloud to the viewer and pass the color handler
    viewer2.addPointCloud(cloud_sliced, cloud_color_handler, "cloud");

    viewer2.addCoordinateSystem(0.3, "cloud", 0);
    viewer2.setBackgroundColor(0.05, 0, 0, 0); // Setting background to white grey

    viewer2.initCameraParameters();
    viewer2.setCameraPosition(1, 2, -1,    0, 0, 1,   -0.1, 0.1, -0.25);
    viewer2.setCameraFieldOfView(0.523599);
    viewer2.setCameraClipDistances(0.00522511, 50); 

    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    while (!viewer2.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer2.spinOnce();
    }
    return 0;

}