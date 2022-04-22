#include "pcl_utils.h"

void loadPCDMap(string pcd_file, LidarPointCloudPtr cloud)
{
    if (pcl::io::loadPCDFile<LidarPoint>(pcd_file, *cloud) == -1)
    {
        cerr << "ERROR: Couldn't read pcd file. Exiting." << endl;
        exit(1);
    }
}

// transform cloud in place
void transform_cloud(LidarPointCloudPtr cloud, Eigen::Affine3f &transform)
{
    LidarPointCloudPtr transformed_cloud(new LidarPointCloud);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    *cloud = *transformed_cloud;
}

// crop cloud and store in cloud_out
void crop_roi(LidarPointCloudConstPtr cloud, 
             LidarPointCloudPtr cloud_out,
             Eigen::Vector3f &centroid_ref,
             double xmin, double xmax,
             double ymin, double ymax,
             double zmin, double zmax) 
{
    // build the condition
    pcl::ConditionAnd<LidarPoint>::Ptr range_cond(new pcl::ConditionAnd<LidarPoint>());
    double x_min, x_max, y_min, y_max, z_min, z_max;
    x_min = centroid_ref[0] + xmin;
    x_max = centroid_ref[0] + xmax;
    y_min = centroid_ref[1] + ymin;
    y_max = centroid_ref[1] + ymax;
    z_min = centroid_ref[2] + zmin;
    z_min = centroid_ref[2] + zmax;
    range_cond->addComparison(pcl::FieldComparison<LidarPoint>::ConstPtr
        (new pcl::FieldComparison<LidarPoint>("x", pcl::ComparisonOps::GT, xmin)));
    range_cond->addComparison(pcl::FieldComparison<LidarPoint>::ConstPtr
        (new pcl::FieldComparison<LidarPoint> ("x", pcl::ComparisonOps::LT, xmax)));
    range_cond->addComparison(pcl::FieldComparison<LidarPoint>::ConstPtr
        (new pcl::FieldComparison<LidarPoint>("y", pcl::ComparisonOps::GT, ymin)));
    range_cond->addComparison(pcl::FieldComparison<LidarPoint>::ConstPtr
        (new pcl::FieldComparison<LidarPoint> ("y", pcl::ComparisonOps::LT, ymax)));
    range_cond->addComparison(pcl::FieldComparison<LidarPoint>::ConstPtr
        (new pcl::FieldComparison<LidarPoint>("z", pcl::ComparisonOps::GT, zmin)));
    range_cond->addComparison(pcl::FieldComparison<LidarPoint>::ConstPtr
        (new pcl::FieldComparison<LidarPoint> ("z", pcl::ComparisonOps::LT, zmax)));
    // build the filter
    pcl::ConditionalRemoval<LidarPoint> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter(*cloud_out);
    remove_invalid(cloud_out);
}

// downsample and do a radius filtering, in place
void preprocess_cloud(LidarPointCloudPtr cloud, double search_radius, int neighbors)
{
    LidarPointCloudPtr cloud_tmp(new LidarPointCloud);
    pcl::RadiusOutlierRemoval<LidarPoint> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(search_radius);
    outrem.setMinNeighborsInRadius(neighbors);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(*cloud_tmp);

    remove_invalid(cloud_tmp);
    *cloud = *cloud_tmp;
}

// modify in place
bool remove_invalid(LidarPointCloudPtr cloud_in) {
    size_t pre_remove = cloud_in->size();
    pcl::PointCloud<LidarPoint>::iterator it = cloud_in->points.begin();
    while (it != cloud_in->points.end()) {
        double x, y, z;
        x = it->x;
        y = it->y;
        z = it->z;
        //cout << "x: " << x << "  y: " << y << "  z: " << z << "  rgb: " << rgb << endl;
        if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z)) {
            it = cloud_in->points.erase(it);
        }
        else
            ++it;
    }
    if (cloud_in->size() < pre_remove)
        cloud_in->width = cloud_in->size();
    return (cloud_in->size() < pre_remove);
}


// assume target cloud is already in the kd_tree
void associate(LidarPointCloudConstPtr source_cloud,
                LidarPointCloudConstPtr target_cloud,
                pcl::KdTreeFLANN<LidarPoint> &kd_tree,
                const Eigen::Vector3f &p_robot,
                vector<float> &d_dists,
                vector<float> &angles)
{

    for (int i = 0; i < source_cloud->size(); i++)
    {
        LidarPoint searchPoint = source_cloud->points[i];
        std::vector<int> pointIdxKNNSearch(1);
        std::vector<float> pointKNNSquaredDistance(1);
        // float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
        if (kd_tree.nearestKSearch(searchPoint, 1, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
        {
            LidarPoint pt = (*target_cloud)[pointIdxKNNSearch[0]];
            Eigen::Vector3f p_target(pt.x, pt.y, pt.z);
            Eigen::Vector3f p_src(searchPoint.x, searchPoint.y, searchPoint.z);
            Eigen::Vector3f d_target, d_src;
            d_target = p_target - p_robot;
            d_dists.push_back(d_target.norm());
            d_src = p_src - p_robot;
            float angle = acos(d_target.dot(d_src) / (d_target.norm() * d_src.norm()));
            angles.push_back(angle);       
        }
    }
}

bool align_pcl_icp(LidarPointCloudConstPtr source_cloud,
                    LidarPointCloudConstPtr target_cloud,
                    LidarPointCloudPtr cloud_aligned,
                    Eigen::Affine3f &tf_out)
{
    pcl::IterativeClosestPoint<LidarPoint, LidarPoint> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    icp.align(*cloud_aligned);

    tf_out.matrix() = icp.getFinalTransformation();
    return icp.hasConverged();
}