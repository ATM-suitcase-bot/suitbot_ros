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

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{

    LidarPointCloudPtr cloud(new LidarPointCloud);
    LidarPointCloudPtr cloud_filtered(new LidarPointCloud);
    LidarPointCloudPtr cloud_sliced(new LidarPointCloud);
    LidarPointCloudPtr cloud_denoised(new LidarPointCloud);
    string filename_1 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/wean.pcd";
    string filename_2 = "/home/tina-laptop/localFiles/research/blaser/blaser_ws/src/blaser_mapping/test_data/wean_processed.pcd";

    if (pcl::io::loadPCDFile<LidarPoint>(filename_1, *cloud) == -1)
    {
        cout << "Couldn't read pcd file." << endl;
        return -1;
    }

    Vector3f A(-12.258408, 1.914654, 1.107991);
    //Vector3f A(-14.726460, -29.710091, 2.804113);
    Vector3f B(10.381905, 12.456957, -1.023135);
    //Vector3f C(-4.977568, 66.388695, -1.027254);
    Vector3f C(-11.208848, 63.320454, -0.476158);

    // a*x+b*y+c*z+d=0
    float a = (B[1]-A[1])*(C[2]-A[2])-(C[1]-A[1])*(B[2]-A[2]);
    float b = (B[2]-A[2])*(C[0]-A[0])-(C[2]-A[2])*(B[0]-A[0]);
    float c = (B[0]-A[0])*(C[1]-A[1])-(C[0]-A[0])*(B[1]-A[1]);
    float d = -(a*A[0]+b*A[1]+c*A[2]);

    // subsample cloud
    cout << "original cloud size: " << cloud->points.size() << endl;

    pcl::VoxelGrid<LidarPoint> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_filtered);

    cout << "filtered cloud size: " << cloud_filtered->points.size() << endl;
/*
    // check distance to plane and filter
    for (int i = 0; i < cloud_filtered->points.size(); i++)
    {
        float dist = abs(a*cloud_filtered->points[i].x + b*cloud_filtered->points[i].y + c*cloud_filtered->points[i].z + d) / sqrt(a*a + b*b + c*c);
        if (dist < 0.5)
            cloud_sliced->points.push_back(cloud_filtered->points[i]);
    }
*/
    // get rid of spurious points in the middle
    // then we use radius filter
    pcl::RadiusOutlierRemoval<LidarPoint> outrem;
    // build the filter
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(2.0);
    outrem.setMinNeighborsInRadius(30);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(*cloud_denoised);

    // project to plane

    // find normal and transform to z=0
    Vector3f n = ((A - C).cross(B - C)).normalized();

    // export to occ map
    LidarPoint minPt, maxPt;
    pcl::getMinMax3D (*cloud_denoised, minPt, maxPt);
    cout << "min x y z: "<< minPt.x << ",  " << minPt.y<< ", " <<minPt.z << endl;
    cout << "max x y z: "<< maxPt.x << ",  " << maxPt.y<< ", " <<maxPt.z << endl;
    cloud_denoised->width = 1;
    cloud_denoised->height = cloud_denoised->points.size();
    pcl::io::savePCDFileASCII (filename_2, *cloud_denoised);

    exit(0);

    // x in  -40 m to 40 m (horizontal), y in -40 to 80 (verticle), z 0
    cv::Mat outimg = cv::Mat::ones(240, 160, CV_8UC1) * 255;
    for (int i = 0; i < cloud_denoised->points.size(); i++)
    {
        float x = cloud_denoised->points[i].x;
        float y = cloud_denoised->points[i].y;
        if (x < 40.0 && x > -40.0 && y >-40.0 && y < 80.0) {
            int x_coord = int(floor((x + 40) / 0.5));
            int y_coord = int(floor((y + 40) / 0.5));
            //cout << "good" << endl;
            //cout << x_coord << ", " << y_coord << endl;
            outimg.at<uchar>(239 - y_coord , x_coord) = 0;
        }
    }
    cv::imwrite("wean_map_grid.jpg", outimg);
    cv::imshow("Gray scale", outimg); 
    cv::waitKey(0);

    // visualize
     pcl::visualization::PCLVisualizer viewer2("my_vis");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<LidarPoint> cloud_color_handler(cloud_denoised, 80, 70, 242);
    // We add the point cloud to the viewer and pass the color handler
    viewer2.addPointCloud(cloud_denoised, cloud_color_handler, "cloud");

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