/**
 * @file localization_node.cpp
 *
 * @brief Use a PF for localization
 *
 * @date 02/05/2022
 *
 * @author Tina Tian (yutian)
 */

#include "localization_node.h"
#include <pcl/filters/voxel_grid.h>

LocalizationNode::LocalizationNode(ros::NodeHandle *nodehandle, parameters_t &_params) : nh(*nodehandle), params(_params), 
                                                                        map_point_cloud_msg(new sensor_msgs::PointCloud2)
{
    ROS_INFO("Initializing Localization Node...");
    prev_odom(0) = 0.0;
    prev_odom(1) = 0.0;
    prev_odom(2) = 0.0;
    cur_odom = prev_odom;
    pf.set_params(params.map_file, params.pcd_file, params.global_map_resolution, 
                  params.fixed_height, 
                  params.pf_init_num_particles_per_grid, params.pf_init_num_particles_total,
                  params.pf_alpha1, params.pf_alpha2, params.pf_alpha3, params.pf_alpha4,
                  params.pf_sigma_hit, params.pf_lambda_short, params.pf_max_range, params.pf_max_span,
                  params.pf_z_hit, params.pf_z_short, params.pf_z_max, params.pf_z_rand);

    LidarPointCloudPtr cloud (new LidarPointCloud);
    loadPCDMap(params.pcd_file, cloud);
    pcl_to_cloud_msg(cloud, map_point_cloud_msg);
    map_point_cloud_msg->header.frame_id = params.global_frame_id;

    initializeSubscribers();
    initializePublishers();
    initializeServices();
}

void LocalizationNode::initializeSubscribers()
{
    point_sub = nh.subscribe(params.LIDAR_SYNC_TOPIC, 1, &LocalizationNode::pointcloudCallback, this);
    odom_sub = nh.subscribe(params.CTRL_TOPIC, 1, &LocalizationNode::odomCallback, this);
}

void LocalizationNode::initializeServices()
{
}

void LocalizationNode::initializePublishers()
{
    particles_pose_pub = nh.advertise<geometry_msgs::PoseArray>(params.PF_PARTICLES_TOPIC, 1, true);
    if (params.publish_point_cloud_rate != 0)
    {
        map_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(params.PCD_MAP_TOPIC, 1, true);
        map_point_cloud_pub_timer = nh.createTimer(ros::Duration(ros::Rate(params.publish_point_cloud_rate)),
                                                    &LocalizationNode::publishMapPointCloud, this);
    }
  
}

bool LocalizationNode::serviceCallback(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response)
{
    ROS_INFO("service callback activated");
    response.success = true; // boring, but valid response info
    response.message = "here is a response string";
    return true;
}

int LocalizationNode::run()
{

    // step 1: init particles in free space. each particle is the center of a submap

    // step 2: rotate the robot > 360 degrees while mapping the env using Livox-LOAM

    // step 3: register local map created by lidar onto each submap (centered around each particle)

    // step 4: update particle position and orientation

    // step 5: transform the sensed local map to each particle's position and compute difference of sensed local map vs each particle's submap

    // step 6: resample the particles

    // step 7: regular PF
    return 0;
}

void LocalizationNode::publishParticles()
{
    /* If the filter is not initialized then exit */
    if (!pf.isInitialized())
        return;

    /* Build the msg based on the particles position and orientation */
    geometry_msgs::PoseArray msg;
    pf.buildParticlesPoseMsg(msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = params.global_frame_id;

    /* Publish particle cloud */
    particles_pose_pub.publish(msg);
}


void LocalizationNode::publishMapPointCloud(const ros::TimerEvent&)
{
    ROS_DEBUG("[%s] Node::publishMapPointCloud()", ros::this_node::getName().data());
    map_point_cloud_msg->header.stamp = ros::Time::now();
    map_point_cloud_pub.publish(*map_point_cloud_msg);
}



void LocalizationNode::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_DEBUG("pointcloudCallback open");

    if (!is_odom)
    {
        ROS_WARN("Odometry transform not received");
        return;
    }

    // check if an update must be performed or not
    if (!checkUpdateThresholds())
        return;

    static const ros::Duration update_interval(1.0 / params.pf_update_rate);
    nextupdate_time = ros::Time::now() + update_interval;

    // apply voxel grid
    LidarPointCloudPtr cloud_src(new LidarPointCloud);
    pcl::fromROSMsg(*msg, *cloud_src);
    pcl::VoxelGrid<LidarPoint> sor;
    sor.setInputCloud(cloud_src);
    sor.setLeafSize(params.voxel_size, params.voxel_size, params.voxel_size);
    LidarPointCloudPtr cloud_down(new LidarPointCloud);
    sor.filter(*cloud_down);
    
    // record time spent

    Eigen::Vector3f delta_odom = cur_odom - prev_odom;
    prev_odom = cur_odom;

    pf.predict(delta_odom[0], delta_odom[1], delta_odom[2]);
    // record time spent

    /* Perform particle update based on current point-cloud */
    pf.update(cloud_down);
    // record time spent

    mean_p = pf.getMean();

    // Update time and transform information
    

    // Do the resampling if needed
    static int n_updates = 0;
    if (++n_updates > params.pf_resample_interval)
    {
        n_updates = 0;
        pf.resample(10);
    }
    // record time spent

    /* Publish particles */
    publishParticles();

    ROS_DEBUG("pointcloudCallback close");
}

void LocalizationNode::odomCallback(const nav_msgs::Odometry &ctrl_in)
{
    ROS_DEBUG("odomCallback open");
    // If the filter is not initialized, initialize it then exit
    if (!pf.isInitialized())
    {
        ROS_WARN("Filter not initialized yet. Initializing particles randomly in free space.");
        pf.init();
        publishParticles();
        return;
    }

    // Update cur odometry
    cur_odom(0) = (float)(ctrl_in.pose.pose.position.x);
    cur_odom(1) = (float)(ctrl_in.pose.pose.position.y);
    cur_odom(2) = (float)tf::getYaw(ctrl_in.pose.pose.orientation);
    
    if (!is_odom) is_odom = true;


    // detect jump? detect error?

    // update mean particle set
    

    // Publish mean particle set and confidence
    
    ROS_DEBUG("odomCallback close");
}

bool LocalizationNode::checkUpdateThresholds()
{
    ROS_DEBUG("Checking for AMCL3D update");

    if (ros::Time::now() < nextupdate_time)
        return false;
    else
        return true;
    
    Eigen::Vector3f delta_odom = cur_odom - prev_odom;

    // Check translation threshold
    if (delta_odom[0]*delta_odom[0] + delta_odom[1]*delta_odom[1] > params.pf_update_dist_threshold)
    {
        ROS_INFO("Translation update");
        return true;
    }

    // Check yaw threshold 
    if (fabs(warpAngle(delta_odom[2])) > params.pf_update_angle_threshold) 
    {
        ROS_INFO("Rotation update");
        return true;
    }
    return false;
}


int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "localization_node"); // node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    parameters_t params;
    params.readParameters(nh);

    ROS_INFO("main: instantiating an object of type LocalizationNode");
    LocalizationNode localizationNode(&nh, params);

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    // ros::spin();
    while (ros::ok())
    {
        ros::spinOnce();
        usleep(100);
    }

    nh.shutdown();
    return 0;
}