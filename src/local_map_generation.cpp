#include "local_map_generation.h"



LocalMapGenerator::LocalMapGenerator(ros::NodeHandle* nodehandle, parameters_t &_params) : 
    nh(*nodehandle), params(_params), point_cloud_map(new LidarPointCloud), it(nh)
{
    ROS_INFO("Initializing Local Map Generator...");
    loadPCDMap(params.pcd_file, point_cloud_map);
    // transform raw pcd to align with our world coordinate
    transform_cloud(point_cloud_map, params.pcd_map_transformation);
    initializeSubscribers();
    initializePublishers();
}


void LocalMapGenerator::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    odom_sub = nh.subscribe(params.CTRL_TOPIC, 1, &LocalMapGenerator::odom_callback, this);
    pointcloud_sub = nh.subscribe(params.LIDAR_SYNC_TOPIC, 1, &LocalMapGenerator::pointcloud_callback, this);
}


void LocalMapGenerator::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    local_map_pub = nh.advertise<suitbot_ros::LocalMapMsg>(params.LOCAL_MAP_TOPIC, 1, true);
    local_map_img_pub = it.advertise(params.LOCAL_MAP_IMAGE_TOPIC, 1, true);
}



void LocalMapGenerator::odom_callback(const nav_msgs::Odometry &ctrl_in)
{
    // ctrl_in gives odom in world frame
    // record current odometry in world frame 
    cur_trans_w[0] = ctrl_in.pose.pose.position.x;
    cur_trans_w[1] = ctrl_in.pose.pose.position.y;
    cur_trans_w[2] = params.fixed_height;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(ctrl_in.pose.pose.orientation, q);
    cur_rot_w = q.cast<float>();
}

void LocalMapGenerator::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // input cloud is in robot frame
    LidarPointCloudPtr cloud_in(new LidarPointCloud);
    pcl::fromROSMsg(*msg, *cloud_in);
    
    // Step 1: TODO preprocess cloud

    // Step 2: transform to map frame
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << cur_trans_w[0], cur_trans_w[1], cur_trans_w[2];
    transform.rotate(cur_rot_w);
        // note: this is tf_w_r
        // Executing the transformation
    transform_cloud(cloud_in, transform);

    // Step 3: align observation locally with the map:
        // 3.1 crop the local area in the map
    LidarPointCloudPtr local_map_cropped(new LidarPointCloud);
    crop_roi(point_cloud_map, local_map_cropped, cur_trans_w, 
                                          params.local_map_xmin, params.local_map_xmax,
                                          params.local_map_ymin, params.local_map_ymax,
                                          params.local_map_zmin, params.local_map_zmax);
        // 3.2 do an ICP, transform cloud_in (should not be far from cur pose)
    LidarPointCloudPtr meas_cloud_aligned(new LidarPointCloud);
    Eigen::Affine3f tf_gt_pred;
    bool res = align_pcl_icp(cloud_in, local_map_cropped, meas_cloud_aligned, tf_gt_pred);
        // 3.3 check how far
    if (res)
    {
        Eigen::Matrix4f T_gt_pred = tf_gt_pred.matrix();
        Eigen::Vector3f dt;
        Eigen::Quaternionf dq;
        T2tq(T_gt_pred, dt, dq);
        cout << "dt norm: " << dt.norm() << ",  dq norm: " << dq.norm() << endl; 
        if (dt.norm() > params.local_icp_dt_thresh || dq.norm() > params.local_icp_dq_thresh)
        {
            ROS_WARN_STREAM("Local ICP too far!!");
        }
        else
        {
            // TODO update cur pose
            
        }
    }
    else // icp failed
    {
        ROS_WARN_STREAM("Local ICP failed, directly using cloud measurement");
        *meas_cloud_aligned = *cloud_in;
    }

    // Step 4: merge the local map and the meas cloud aligned (with downsampling)
    LidarPointCloudPtr cloud_merged(new LidarPointCloud);
    for (int i = 0; i < meas_cloud_aligned->points.size(); i+=5)
    {
        LidarPoint pt_tmp;
        pt_tmp.x = meas_cloud_aligned->points[i].x;
        pt_tmp.y = meas_cloud_aligned->points[i].y;
        pt_tmp.z = meas_cloud_aligned->points[i].z;
        cloud_merged->push_back(pt_tmp);
    }
    for (int i = 0; i < local_map_cropped->points.size(); i+=5)
    {
        LidarPoint pt_tmp;
        pt_tmp.x = local_map_cropped->points[i].x;
        pt_tmp.y = local_map_cropped->points[i].y;
        pt_tmp.z = local_map_cropped->points[i].z;
        cloud_merged->push_back(pt_tmp);
    }

    // Step 5: transform to robot frame by premultiplying tf_r_w
    Eigen::Matrix4f tf_r_w = invT(transform.matrix());
    Eigen::Affine3f transform_r_w = Eigen::Affine3f::Identity();
    transform_r_w.matrix() = tf_r_w;
    transform_cloud(cloud_merged, transform_r_w);
    cloud_merged->is_dense = false;

    // Step 6: threshold to find out obstacles and project to occ grid:
    OccupancyMap local_occ_map;
        // 6.1 create a 2d array and resize
    vector<vector<int>> occ;
    LidarPoint minPt, maxPt;
    pcl::getMinMax3D(*cloud_merged, minPt, maxPt); // we care about y bounds. we fix lookahead x dist
    float miny = minPt.y;
    float maxy = maxPt.y;
    float maxx = params.local_map_lookahead;
    float minx = -2.0; // we don't care too much about stuff behind us
    int rows, cols;
    cols = (int)(fabs(maxy - miny) / params.local_map_resolution);
    rows = (int)(fabs(maxx - minx) / params.local_map_resolution);
    occ.resize(rows, std::vector<int>(cols));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) 
            occ[i][j] = UNKNOWN;
        // 6.2 bound the 2d array with OCCUPIED
    bound_2d_array(occ, OCCUPIED);
        // 6.3 define obstacles to be: within a certain height, there is stuff
    for (int i = 0; i < cloud_merged->points.size(); i++)
    {
        float z = cloud_merged->points[i].z;
        if (z < params.obstacle_zmin || z > params.obstacle_zmax)
            continue;
        LidarPoint tmp_pt;
        tmp_pt.x = cloud_merged->points[i].x - minx;
        tmp_pt.y = cloud_merged->points[i].x - miny;
        int x_idx = min(max((int)(tmp_pt.x / params.local_map_resolution), 0), rows-1);
        int y_idx = min(max((int)(tmp_pt.y / params.local_map_resolution), 0), cols-1);
        occ[x_idx][y_idx] = OCCUPIED;
    } 
        // 6.4 localflood fill the free space with FREE
    int robot_x_idx = min(max((int)(-minx / params.local_map_resolution), 0), rows-1);
    int robot_y_idx = min(max((int)(-miny / params.local_map_resolution), 0), cols-1);
    floodFill(occ, rows, cols, robot_x_idx, robot_y_idx, OCCUPIED, FREE);
        // 6.5 construct occ map
    local_occ_map.initOccupancyGridMap(occ, params.local_map_resolution);

    // Step 7: publish local occ map
    suitbot_ros::LocalMapMsg map_msg;
    map_msg.header.stamp = ros::Time::now();
    map_msg.rows = rows;
    map_msg.cols = cols;
    map_msg.robot_x_idx = robot_x_idx;
    map_msg.robot_y_idx = robot_y_idx;
    map_msg.cells = local_occ_map.flatten();
    local_map_pub.publish(map_msg);

    sensor_msgs::ImagePtr image;
    local_occ_map.toImageMsg(image);
    local_map_img_pub.publish(image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map_generator");
    ros::NodeHandle nh;

    parameters_t params;
    params.readParameters(nh);

    ROS_INFO_STREAM("main: instantiating an object of type LocalMapGenerator");
    LocalMapGenerator localMapGenerator(&nh, params);

    ROS_INFO("Local map generator: main going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}