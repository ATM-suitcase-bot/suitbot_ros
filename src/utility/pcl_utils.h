#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_

#include <math.h>
#include "../lidar.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/angles.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>

using namespace std;

void loadPCDMap(string pcd_file, LidarPointCloudPtr cloud);


// transform cloud in place
void transform_cloud(LidarPointCloudPtr cloud, Eigen::Affine3f &transform);

// crop cloud and store in cloud_out
void crop_roi(LidarPointCloudConstPtr cloud, 
             LidarPointCloudPtr cloud_out,
             Eigen::Vector3f &centroid_ref,
             double xmin, double xmax,
             double ymin, double ymax,
             double zmin, double zmax);

// downsample and do a radius filtering, in place
void preprocess_cloud(LidarPointCloudPtr cloud, double search_radius, int neighbors);


// modify in place
bool remove_invalid(LidarPointCloudPtr cloud_in);

// assume target cloud is already in the kd_tree
void associate(LidarPointCloudConstPtr source_cloud,
                LidarPointCloudConstPtr target_cloud,
                pcl::KdTreeFLANN<LidarPoint> &kd_tree,
                const Eigen::Vector3f &p_robot,
                vector<float> &d_dists,
                vector<float> &angles);


bool align_pcl_icp(LidarPointCloudConstPtr source_cloud,
                    LidarPointCloudConstPtr target_cloud,
                    LidarPointCloudPtr cloud_aligned,
                    Eigen::Affine3f &tf_out);

#endif