#include "lidar_parse.h"

LidarParse::LidarParse(ros::NodeHandle* nodehandle, parameters_t &_params) : 
    nh(*nodehandle), params(_params), point_cloud_map(new LidarPointCloud), it(nh)

{
    ROS_INFO("Initializing Local Map Generator...");
    // transform raw pcd to align with our world coordinate
    
    initializeSubscribers();
    initializePublishers();
}


void LidarParse::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    pointcloud_sub = nh.subscribe(params.LIDAR_ORIGINAL_TOPIC, 1, &LidarParse::pointcloud_callback, this);
}


void LidarParse::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    local_map_pub = nh.advertise<suitbot_ros::LocalMapMsg>("/suitbot/local_obs", 1, true);
    local_map_img_pub = it.advertise("/suitbot/local_img", 1, true);

}



void LidarParse::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // input cloud is in robot frame
    LidarPointCloudPtr cloud_in(new LidarPointCloud);
    pcl::fromROSMsg(*msg, *cloud_in);
    
    // Step 1: TODO preprocess cloud
    cloud_in->is_dense = false;

    // Step 6: threshold to find out obstacles and project to occ grid:
    OccupancyMap local_occ_map;
        // 6.1 create a 2d array and resize
    vector<vector<int>> occ;
    LidarPoint minPt, maxPt;
    pcl::getMinMax3D(*cloud_in, minPt, maxPt); // we care about y bounds. we fix lookahead x dist
    float miny = minPt.y;
    float maxy = maxPt.y;
    //ROS_INFO_STREAM("miny: " << miny << " maxy: " << maxy);
    float maxx = params.local_map_lookahead;
    float minx = -2.0; // we don't care too much about stuff behind us
    int rows, cols;
    float padding = 1.0; // total padding (left + right) or (top + bot)
    cols = (int)((fabs(maxy - miny) + padding)/ params.local_map_resolution);
    rows = (int)((fabs(maxx - minx) + padding) / params.local_map_resolution);
    //ROS_INFO_STREAM("rows (x): " << rows << ", cols (y): " << cols);
    occ.resize(rows, std::vector<int>(cols));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) 
            occ[i][j] = UNKNOWN;
        // 6.2 bound the 2d array with OCCUPIED
    bound_2d_array(occ, OCCUPIED);
        // 6.3 define obstacles to be: within a certain height, there is stuff
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        float z = cloud_in->points[i].z;
        if (z < params.obstacle_zmin || z > params.obstacle_zmax)
            continue;
        LidarPoint tmp_pt;
        tmp_pt.x = cloud_in->points[i].x - minx + padding/2.0;
        tmp_pt.y = cloud_in->points[i].y - miny + padding/2.0;
        int x_idx = min(max((int)(tmp_pt.x / params.local_map_resolution), 0), rows-1);
        int y_idx = min(max((int)(tmp_pt.y / params.local_map_resolution), 0), cols-1);
        occ[x_idx][y_idx] = OCCUPIED;
    } 
        // 6.4 localflood fill the free space with FREE
    int robot_x_idx = min(max((int)((-minx + padding/2.0) / params.local_map_resolution), 0), rows-1);
    int robot_y_idx = min(max((int)((-miny + padding/2.0) / params.local_map_resolution), 0), cols-1);
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

    //sensor_msgs::ImagePtr image = new (sensor_msgs::Image);
    //local_occ_map.toImageMsg(image);
    //local_map_img_pub.publish(image);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_parse");
    ros::NodeHandle nh;

    parameters_t params;
    params.readParameters(nh);

    ROS_INFO_STREAM("main: instantiating an object of type LocalMapGenerator");
    LidarParse lidarParse(&nh, params);

    ROS_INFO("Local map generator: main going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}

