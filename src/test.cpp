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
    float a, b, c, d, a1, b1, c1, d1, a2, b2, c2, d2;
    Vector3f A(14.573961, 6.168147, 0.349627);
    Vector3f B(14.847487, 4.365918, 2.187052);
    Vector3f C(15.048866, 3.578148, -0.526846);

    Vector3f A1(11.809673, 6.237759, 1.998423);
    Vector3f B1(10.139780, 4.985734, 2.017425);
    Vector3f C1(14.956503, 3.157286, 2.310788);

    Vector3f A2(15.256092, 2.381920, -0.422705);
    Vector3f B2(14.055065, 4.319431, -0.555551);
    Vector3f C2(11.813416, 3.272061, -0.612957);

    three_point_plane(A, B, C, a, b, c, d);

    three_point_plane(A1, B1, C1, a1, b1, c1, d1);

    three_point_plane(A2, B2, C2, a2, b2, c2, d2);

    int num_sample = 10000;
    std::random_device rd;
    std::uniform_real_distribution<float> dist_z(-0.8, 2.3);
    std::uniform_real_distribution<float> dist_y(2.5, 8.5);

    std::uniform_real_distribution<float> dist_x1(10.2, 14.9);
    std::uniform_real_distribution<float> dist_y1(2.5, 8.5);

    std::uniform_real_distribution<float> dist_x3(11.3, 14.9);
    std::uniform_real_distribution<float> dist_y3(2.5, 8);


    float a11, b11, c11, d11;
    Vector3f A11(-5.214748, 11.929585, -1.840274);
    Vector3f B11(-0.744121, 12.462557, -0.955106);
    Vector3f C11(-1.086573, 12.412640, 1.551559);
    three_point_plane(A11, B11, C11, a11, b11, c11, d11);
    std::uniform_real_distribution<float> dist_x11(-5.25, -1.1);
    std::uniform_real_distribution<float> dist_z11(-1.8, 1.6);
    
    float a12, b12, c12, d12;
    Vector3f A12(-0.912135, 12.386212, 1.618194);
    Vector3f B12(-5.446338, 12.039245, 0.972815);
    Vector3f C12(-0.721194, 9.648919, 1.670953);
    three_point_plane(A12, B12, C12, a12, b12, c12, d12);
    std::uniform_real_distribution<float> dist_x12(-5.4, -1.05);
    std::uniform_real_distribution<float> dist_y12(9.75, 12.4);

    float a13, b13, c13, d13;
    Vector3f A13(-4.177891, 9.750760, 0.982951);
    Vector3f B13(-4.237700, 9.619115, -1.720509);
    Vector3f C13(-0.617680, 9.779168, 0.445144);
    three_point_plane(A13, B13, C13, a13, b13, c13, d13);
    std::uniform_real_distribution<float> dist_x13(-5.1, -0.55);
    std::uniform_real_distribution<float> dist_z13(-1.6, 1.6);

    float a14, b14, c14, d14;
    Vector3f A14(-4.155771, 9.650693, -1.719898);
    Vector3f B14(-0.368354, 9.749048, -1.005483);
    Vector3f C14(-0.751506, 12.378446, -0.993207);
    three_point_plane(A14, B14, C14, a14, b14, c14, d14);
    std::uniform_real_distribution<float> dist_x14(-5.3, -0.8);
    std::uniform_real_distribution<float> dist_y14(9.65, 12.4);





    float a21, b21, c21, d21;
    Vector3f A21(1.951173, 12.808295, -0.731401);
    Vector3f B21(7.380652, 13.843306, -0.824956);
    Vector3f C21(7.799297, 14.008073, 1.680046);
    three_point_plane(A21, B21, C21, a21, b21, c21, d21);
    std::uniform_real_distribution<float> dist_x21(1.8, 7.8);
    std::uniform_real_distribution<float> dist_z21(-0.8, 1.7);
    
    float a22, b22, c22, d22;
    Vector3f A22(1.842192, 12.787577, 1.765168);
    Vector3f B22(8.963876, 11.806077, 1.760865);
    Vector3f C22(7.888090, 14.063223, 1.676685);
    three_point_plane(A22, B22, C22, a22, b22, c22, d22);
    std::uniform_real_distribution<float> dist_x22(1.9, 8.9);
    std::uniform_real_distribution<float> dist_y22(10.0, 14.1);

    float a23, b23, c23, d23;
    Vector3f A23(1.810980, 10.142950, 1.273992);
    Vector3f B23(8.919257, 11.549464, -0.800122);
    Vector3f C23(8.963876, 11.806077, 1.760865);
    three_point_plane(A23, B23, C23, a23, b23, c23, d23);
    std::uniform_real_distribution<float> dist_x23(1.7, 9.0);
    std::uniform_real_distribution<float> dist_z23(-0.8, 1.8);

    float a24, b24, c24, d24;
    Vector3f A24(1.978306, 12.790258, -0.793569);
    Vector3f B24(7.202655, 13.810786, -0.820488);
    Vector3f C24(8.955860, 11.504584, -0.909936);
    three_point_plane(A24, B24, C24, a24, b24, c24, d24);
    std::uniform_real_distribution<float> dist_x24(1.8, 9.0);
    std::uniform_real_distribution<float> dist_y24(10.0, 14.1);

    for (int i = 0; i < num_sample; i++)
    {
        cout << "sample id: " << i << endl;
        float x = 0;
        float y = dist_y(rd);
        float z = dist_z(rd);
        x = (-d - b*y - c*z) / a;
        LidarPoint pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 

        x = dist_x1(rd);
        y = dist_y1(rd);
        z = (-d1 - b1*y - a1*x) / c1;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 

        x = dist_x3(rd);
        y = dist_y3(rd);
        z = (-d2 - b2*y - a2*x) / c2;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 

        // 11 (front)
        x = dist_x11(rd);
        z = dist_z11(rd);
        y = (-d11 - a11*x - c11*z) / b11;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 

        // 12 (top)
        x = dist_x12(rd);
        y = dist_y12(rd);
        z = (-d12 - b12*y - a12*x) / c12;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 

        // 13 (back)
        x = dist_x13(rd);
        z = dist_z13(rd);
        y = (-d13 - a13*x - c13*z) / b13;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt);

        // 14 (bot)
        x = dist_x14(rd);
        y = dist_y14(rd);
        z = (-d14 - b14*y - a14*x) / c14;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 







        // 21 (front)
        x = dist_x21(rd);
        z = dist_z21(rd);
        y = (-d21 - a21*x - c21*z) / b21;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 

        // 22 (top)
        x = dist_x22(rd);
        y = dist_y22(rd);
        z = (-d22 - b22*y - a22*x) / c22;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 

        // 23 (back)
        x = dist_x23(rd);
        z = dist_z23(rd);
        y = (-d23 - a23*x - c23*z) / b23;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt);

        // 24 (bot)
        x = dist_x24(rd);
        y = dist_y24(rd);
        z = (-d24 - b24*y - a24*x) / c24;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        cloud->points.push_back(pt); 
    }




    cloud->width = 1;
    cloud->height = cloud->points.size();



    pcl::io::savePCDFileASCII("/home/tina/Documents/atm_ws/src/suitbot_ros/data/wean_map_good_filled.pcd", *cloud);

    return 0;
}
