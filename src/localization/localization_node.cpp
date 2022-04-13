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

LocalizationNode::LocalizationNode(ros::NodeHandle *nodehandle, parameters_t &_params) : nh(*nodehandle), params(_params)
{
    ROS_INFO("Initializing Corner Extractor...");
    initializeSubscribers();
    initializePublishers();
    initializeServices();
}

void LocalizationNode::initializeSubscribers()
{
    point_sub = nh.subscribe("/livox/lidar", 1, &LocalizationNode::pointcloudCallback, this);
    odom_sub = nh.subscribe("/odometry", 1, &LocalizationNode::odomCallback, this);
}

void LocalizationNode::initializeServices()
{
}

void LocalizationNode::initializePublishers()
{
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

void LocalizationNode::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_DEBUG("pointcloudCallback open");

    if (!is_odom_)
    {
        ROS_WARN("Odometry transform not received");
        return;
    }

    // check if an update must be performed or not
    if (!checkUpdateThresholds())
        return;

    static const ros::Duration update_interval(1.0 / parameters_.update_rate_);
    nextupdate_time_ = ros::Time::now() + update_interval;

    // apply voxel grid
    LidarPointCloudPtr cloud_src(new LidarPointCloud);
    pcl::fromROSMsg(*msg, *cloud_src);
    pcl::VoxelGrid<LidarPoint> sor;
    sor.setInputCloud(cloud_src);
    sor.setLeafSize(parameters_.voxel_size_, parameters_.voxel_size_, parameters_.voxel_size_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_down);
    cloud_down->header = cloud_src->header;
    sensor_msgs::PointCloud2 cloud_down_msg;
    pcl::toROSMsg(*cloud_down, cloud_down_msg);
    cloud_filter_pub_.publish(cloud_down_msg);
    // record time spent

    /* Perform particle prediction based on odometry */
    odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;
    const double delta_x = odom_increment_tf_.getOrigin().getX();
    const double delta_y = odom_increment_tf_.getOrigin().getY();
    const double delta_z = odom_increment_tf_.getOrigin().getZ();
    const double delta_a = getYawFromTf(odom_increment_tf_);

    pf.predict(parameters_.odom_x_mod_, parameters_.odom_y_mod_, parameters_.odom_z_mod_, parameters_.odom_a_mod_,
               delta_x, delta_y, delta_z, delta_a);
    // record time spent

    /* Perform particle update based on current point-cloud */
    pf.update(grid3d_, cloud_down, range_data, parameters_.alpha_, parameters_.sensor_range_, roll_, pitch_);
    // record time spent

    mean_p = pf.getMean();

    /* Update time and transform information */
    lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

    /* Do the resampling if needed */
    static int n_updates = 0;
    if (++n_updates > parameters_.resample_interval_)
    {
        n_updates = 0;
        pf.resample();
    }
    // record time spent

    /* Publish particles */
    publishParticles();

    ROS_DEBUG("pointcloudCallback close");
}

void LocalizationNode::odomCallback(const geometry_msgs::TransformStampedConstPtr &msg)
{
    ROS_DEBUG("odomCallback open");

    base_2_odom_tf_.setOrigin(
        tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
    base_2_odom_tf_.setRotation(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                                               msg->transform.rotation.z, msg->transform.rotation.w));

    /* If the filter is not initialized then exit */
    if (!pf_.isInitialized())
    {
        ROS_WARN("Filter not initialized yet, waiting for initial pose.");
        if (parameters_.set_initial_pose_)
        {
            tf::Transform init_pose;
            init_pose.setOrigin(tf::Vector3(parameters_.init_x_, parameters_.init_y_, parameters_.init_z_));
            init_pose.setRotation(tf::Quaternion(0.0, 0.0, sin(parameters_.init_a_ * 0.5), cos(parameters_.init_a_ * 0.5)));
            setInitialPose(init_pose, parameters_.init_x_dev_, parameters_.init_y_dev_, parameters_.init_z_dev_,
                           parameters_.init_a_dev_);
        }
        return;
    }

    /* Update roll and pitch from odometry */
    double yaw;
    base_2_odom_tf_.getBasis().getRPY(roll_, pitch_, yaw);

    static tf::TransformBroadcaster tf_br;
    tf_br.sendTransform(
        tf::StampedTransform(base_2_odom_tf_, ros::Time::now(), parameters_.odom_frame_id_, parameters_.base_frame_id_));

    if (!is_odom_)
    {
        is_odom_ = true;

        lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
        lastodom_2_world_tf_ = initodom_2_world_tf_;
    }

    static bool has_takenoff = false;
    if (!has_takenoff)
    {
        ROS_WARN("Not <<taken off>> yet");

        /* Check takeoff height */
        has_takenoff = base_2_odom_tf_.getOrigin().getZ() > parameters_.take_off_height_;

        lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
        lastodom_2_world_tf_ = initodom_2_world_tf_;

        lastmean_p_ = mean_p_; // for not 'jumping' whenever has_takenoff is true */
    }
    else
    {
        /* Check if AMCL went wrong (nan, inf) */
        if (std::isnan(mean_p_.x) || std::isnan(mean_p_.y) || std::isnan(mean_p_.z) || std::isnan(mean_p_.a))
        {
            ROS_WARN("AMCL NaN detected");
            amcl_out_ = true;
        }
        if (std::isinf(mean_p_.x) || std::isinf(mean_p_.y) || std::isinf(mean_p_.z) || std::isinf(mean_p_.a))
        {
            ROS_WARN("AMCL Inf detected");
            amcl_out_ = true;
        }

        /* Check jumps */
        if (fabs(mean_p_.x - lastmean_p_.x) > 1.)
        {
            ROS_WARN("AMCL Jump detected in X");
            amcl_out_ = true;
        }
        if (fabs(mean_p_.y - lastmean_p_.y) > 1.)
        {
            ROS_WARN("AMCL Jump detected in Y");
            amcl_out_ = true;
        }
        if (fabs(mean_p_.z - lastmean_p_.z) > 1.)
        {
            ROS_WARN("AMCL Jump detected in Z");
            amcl_out_ = true;
        }
        if (fabs(mean_p_.a - lastmean_p_.a) > 1.)
        {
            ROS_WARN("AMCL Jump detected in Yaw");
            amcl_out_ = true;
        }

        if (!amcl_out_)
        {
            tf::Transform base_2_world_tf;
            base_2_world_tf.setOrigin(tf::Vector3(mean_p_.x, mean_p_.y, mean_p_.z));
            tf::Quaternion q;
            q.setRPY(roll_, pitch_, mean_p_.a);
            base_2_world_tf.setRotation(q);

            base_2_world_tf = base_2_world_tf * lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

            lastmean_p_ = mean_p_;

            lastbase_2_world_tf_ = base_2_world_tf;
            lastodom_2_world_tf_ = base_2_world_tf * base_2_odom_tf_.inverse();

            amcl_out_lastbase_2_odom_tf_ = lastupdatebase_2_odom_tf_;
        }
        else
        {
            lastbase_2_world_tf_ = lastbase_2_world_tf_ * amcl_out_lastbase_2_odom_tf_.inverse() * base_2_odom_tf_;
            amcl_out_lastbase_2_odom_tf_ = base_2_odom_tf_;
        }
    }

    /* Publish transform */
    geometry_msgs::TransformStamped odom_2_base_tf;
    odom_2_base_tf.header.stamp = msg->header.stamp;
    odom_2_base_tf.header.frame_id = parameters_.global_frame_id_;
    odom_2_base_tf.child_frame_id = parameters_.base_frame_id_;
    odom_2_base_tf.transform.translation.x = lastbase_2_world_tf_.getOrigin().getX();
    odom_2_base_tf.transform.translation.y = lastbase_2_world_tf_.getOrigin().getY();
    odom_2_base_tf.transform.translation.z = lastbase_2_world_tf_.getOrigin().getZ();
    odom_2_base_tf.transform.rotation.x = lastbase_2_world_tf_.getRotation().getX();
    odom_2_base_tf.transform.rotation.y = lastbase_2_world_tf_.getRotation().getY();
    odom_2_base_tf.transform.rotation.z = lastbase_2_world_tf_.getRotation().getZ();
    odom_2_base_tf.transform.rotation.w = lastbase_2_world_tf_.getRotation().getW();
    odom_base_pub_.publish(odom_2_base_tf);

    tf_br.sendTransform(tf::StampedTransform(lastodom_2_world_tf_, ros::Time::now(), parameters_.global_frame_id_,
                                             parameters_.odom_frame_id_));

    ROS_DEBUG("odomCallback close");
}

bool LocalizationNode::checkUpdateThresholds()
{
    ROS_DEBUG("Checking for AMCL3D update");

    if (ros::Time::now() < nextupdate_time_)
        return false;

    odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

    /* Check translation threshold */
    if (odom_increment_tf_.getOrigin().length() > parameters_.d_th_)
    {
        ROS_INFO("Translation update");
        return true;
    }

    /* Check yaw threshold */
    double yaw, pitch, roll;
    odom_increment_tf_.getBasis().getRPY(roll, pitch, yaw);
    if (fabs(yaw) > parameters_.a_th_)
    {
        ROS_INFO("Rotation update");
        return true;
    }

    return false;
}

void LocalizationNode::setInitialPose(const tf::Transform &init_pose, const float x_dev, const float y_dev, const float z_dev,
                                      const float a_dev)
{
    initodom_2_world_tf_ = init_pose;

    const tf::Vector3 t = init_pose.getOrigin();

    const float x_init = t.x();
    const float y_init = t.y();
    const float z_init = t.z();
    const float a_init = static_cast<float>(getYawFromTf(init_pose));

    pf_.init(parameters_.num_particles_, x_init, y_init, z_init, a_init, x_dev, y_dev, z_dev, a_dev);

    mean_p_ = pf_.getMean();
    lastmean_p_ = mean_p_;

    /* Extract TFs for future updates */
    /* Reset lastupdatebase_2_odom_tf_ */
    lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

    /* Publish particles */
    publishParticles();
}

double LocalizationNode::getYawFromTf(const tf::Transform &tf)
{
    double yaw, pitch, roll;
    tf.getBasis().getRPY(roll, pitch, yaw);

    return yaw;
}

int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "initializationNode"); // node name

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