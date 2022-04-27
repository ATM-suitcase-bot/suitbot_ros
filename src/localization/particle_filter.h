/**
 * @file particle_filter.h
 *
 * @brief 2.5D particle filter class
 *  
 * @date 04/10/2022
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <random>

#include <math.h>
#include <float.h>
#include <stdio.h>
#include "../lidar.h"
#include <bits/stdc++.h>

#include "../occupancy_map.h"
#include "../utility/utils.h"
#include "../utility/pcl_utils.h"
#include "../utility/tic_toc.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

using namespace std;

struct Particle
{
	float x;
	float y;
	float theta;
	float weight;

    Particle() : x(0), y(0), theta(0), weight(1) {}

    Particle(float _x, float _y, float _theta, float _weight) : x(_x), y(_y), theta(_theta), weight(_weight) {}
};

class ParticleFilter
{
public:
	// Number of particles to draw
	int initial_num_particles_per_grid = 3;

	int initial_num_particles_total = 20000;

	float fixed_height = -2.25;

	// Set of current particles
	vector<Particle> particles;

	// motion model parameters
	float alpha1 = 0.0001, alpha2 = 0.0001, alpha3 = 0.008, alpha4 = 0.008;

	// sensor model parameters
	float sigma_hit = 50.0, lambda_short = 0.1, max_range = 150.0, max_span = 5.0;
	float z_hit = 10.0, z_short = 0.1, z_max = 0.1, z_rand = 100;

	// 2d map
	OccupancyMap grid_map;

	//3d map
	LidarPointCloudPtr point_cloud_map;

    float dist_thresh = 10;
	pcl::KdTreeFLANN<LidarPoint> kdtree;




	// Constructor
	// @param Number of particles
	ParticleFilter() : point_cloud_map(new LidarPointCloud) {}

	ParticleFilter(string map_file, string pcd_file);

	ParticleFilter(string map_file, string pcd_file, 
					float fixed_height_, int init_num_per_grid, int init_num_total);

	ParticleFilter(string map_file, string pcd_file, 
				   float fixed_height_, int init_num_per_grid, int init_num_total, 
				   float _alpha1, float _alpha2, float _alpha3, float _alpha4,
				   float _sigma_hit, float _lambda_short, float _max_range, float _max_span,
				   float _z_hit, float _z_short, float _z_max, float _z_rand);

	// Destructor
	~ParticleFilter() {}



	void set_params(
                   string map_file, string pcd_file, float resolution,
				   float fixed_height_, int init_num_per_grid, int init_num_total, 
				   float _alpha1, float _alpha2, float _alpha3, float _alpha4,
				   float _sigma_hit, float _lambda_short, float _max_range, float _max_span,
				   float _z_hit, float _z_short, float _z_max, float _z_rand);

    bool isInitialized() const  { return initialized_; }

    Particle getMean() const { return mean_; }



	/**
	 * @brief This function implements the PF init stage.
    */
	void init();

	/**
	 * @brief This function implements the PF prediction stage.
	* 		  (Translation in X, Y and Z in meters and yaw angle incremenet in rad.)
	*
	* @param delta_x Thresholds of x-axis position in prediction.
	* @param delta_y Thresholds of y-axis position in prediction.
	* @param delta_theta Thresholds of yaw angle orientation in prediction.
	*
	* It calculates the increase that has occurred in the odometry and makes predictions of where it is possible that the
	* UAV is, taking into account selected thresholds.
   */
	void predict(const Eigen::Vector3f &cur_odom, const Eigen::Vector3f &prev_odom);

	/** 
	 * @brief This function implements the PF update stage.
	*
	* @param cloud Point cloud from the robot view.
	*
	* It takes the positions of the particles to see if they are on the map. Then, it evaluates the weight of the
	* particle according to the point cloud. Finally, it normalizes the weights
	* for all particles and finds the average for the composition of the robot pose.
   */
	void update(LidarPointCloudConstPtr cloud_in);

	float computeCloudWeight(LidarPointCloudConstPtr cloud, const vector<float> &measurements, 
							 const float px, const float py, const float pth);



	/**
	 * @brief Resample particles - low variance
	 */
	void resample(int num_to_sample);

	void resample_basic();

	/**
	 * @brief Writes particle positions to a file.
	 * @param filename: File to write particle positions to.
	 */
	void write(string filename);

	void buildParticlesPoseMsg(geometry_msgs::PoseArray& msg) const;

	void buildParticleMsg(geometry_msgs::PoseStamped& msg) const;


private:
    Particle mean_;           /*!< Particle to show the mean of all the particles */

    bool initialized_{ false }; /*!< To indicate the initialition of the filter */

    std::random_device rd_{};  /*!< Random device */
    std::mt19937 generator_{rd_()}; /*!< Generator of random values */


	unordered_set<pair<int, int>, hash_pair> sample_area;

    /** 
    * @brief To generate the random value by the Gaussian distribution.
    *
    * @param mean Average of the distribution.
    * @param sigma Desviation of the distribution.
    * @return <b>float</b> - Random value.
    */
    float ranGaussian(const float mean, const float sigma);

    /**
    *  @brief To generate the random between two values.
    *
    * @param range_from Lower end of range.
    * @param range_to Upper end of range.
    * @return <b>float</b> - Random value.
    */
    float rngUniform(const float range_from, const float range_to);

	void sampleParticlesUniform(const float from_x, const float from_y,
                                            const float to_x, const float to_y,
											const int num, vector<Particle> &out_particles);

	void add_sample_area(float sample_area_width, pair<int, int> indices);

};



#endif /* PARTICLE_FILTER_H_ */