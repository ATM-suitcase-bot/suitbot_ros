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

#include "helper_functions.h"
#include <math.h>
#include <float.h>
#include <stdio.h>
#include "../lidar.h"

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
	int initial_num_particles_per_grid;

	int initial_num_particles_total;

	float fixed_height = -2.25;

	// Set of current particles
	vector<Particle> particles;

	// motion model parameters
	double alpha1 = 0.0001, alpha2 = 0.0001, alpha3 = 0.008, alpha4 = 0.008;

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
	ParticleFilter() : initial_num_particles_per_grid(50), initial_num_particles_total(10000), point_cloud_map(new LidarPointCloud), generator_(rd_()) {}

	ParticleFilter(string map_file, string pcd_file) : initial_num_particles_per_grid(50), initial_num_particles_total(10000), point_cloud_map(new LidarPointCloud), generator_(rd_()) 
	{
		int res = grid_map.initOccupancyGridMap(map_file);
		if (res < 0)
		{
			std::cerr << "ERROR: Cannot create occupancy map! Exiting." << endl;
			exit(1);
		}
		loadPCDMap(pcd_file, point_cloud_map);
		kdtree.setInputCloud(point_cloud_map);	
	}

	ParticleFilter(string map_file, string pcd_file, float fixed_height_, int init_num_per_grid, int init_num_total) : fixed_height(fixed_height_), initial_num_particles_per_grid(init_num_per_grid), initial_num_particles_total(init_num_total), point_cloud_map(new LidarPointCloud), generator_(rd_()) {

		int res = grid_map.initOccupancyGridMap(map_file);
		if (res < 0)
		{
			std::cerr << "ERROR: Cannot create occupancy map! Exiting." << endl;
			exit(1);
		}
		loadPCDMap(pcd_file, point_cloud_map);
		kdtree.setInputCloud(point_cloud_map);		
	}

	ParticleFilter(string map_file, string pcd_file, 
				   float fixed_height_, int init_num_per_grid, int init_num_total, 
				   double _alpha1, double _alpha2, double _alpha3, double _alpha4,
				   float _sigma_hit, float _lambda_short, float _max_range, float _max_span,
				   float _z_hit, float _z_short, float _z_max, float _z_rand) : 
				   fixed_height(fixed_height_), 
				   initial_num_particles_per_grid(init_num_per_grid), 
				   initial_num_particles_total(init_num_total), 
				   alpha1(_alpha1), alpha2(_alpha2), alpha3(_alpha3), alpha4(_alpha4), 
				   sigma_hit(_sigma_hit), lambda_short(_lambda_short), max_range(_max_range), max_span(_max_span),
				   z_hit(_z_hit), z_short(_z_short), z_max(_z_max), z_rand(_z_rand),
				   point_cloud_map(new LidarPointCloud), generator_(rd_()) {

		int res = grid_map.initOccupancyGridMap(map_file);
		if (res < 0)
		{
			std::cerr << "ERROR: Cannot create occupancy map! Exiting." << endl;
			exit(1);
		}
		loadPCDMap(pcd_file, point_cloud_map);
		kdtree.setInputCloud(point_cloud_map);		
	}

	// Destructor
	~ParticleFilter() {}

	void set_params(string map_file, string pcd_file, float resolution,
				   float fixed_height_, int init_num_per_grid, int init_num_total);

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
	void predict(const double delta_x, const double delta_y, const double delta_theta);

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


private:
    Particle mean_;           /*!< Particle to show the mean of all the particles */

    bool initialized_{ false }; /*!< To indicate the initialition of the filter */

    std::random_device rd_;  /*!< Random device */
    std::mt19937 generator_; /*!< Generator of random values */

    /** 
    * @brief To generate the random value by the Gaussian distribution.
    *
    * @param mean Average of the distribution.
    * @param sigma Desviation of the distribution.
    * @return <b>float</b> - Random value.
    */
    float ranGaussian(const double mean, const double sigma);

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

};



#endif /* PARTICLE_FILTER_H_ */