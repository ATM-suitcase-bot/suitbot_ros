/**
 * @file particle_filter.h
 *
 * @brief 2.5D particle filter class
 *  
 * @date 04/10/2022
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"
#include <math.h>
#include <float.h>
#include <stdio.h>

#include <Eigen/Dense>

using namespace std;

struct Particle
{
	double x;
	double y;
	double theta;
	double weight;

    Particle() : x(0), y(0), theta(0), weight(0) {}

    Particle(double _x, double _y, double _theta, double _weight) : x(_x), y(_y), theta(_theta), weight(_weight) {}
};

class ParticleFilter
{
	// Number of particles to draw
	int num_particles;


	// Vector of weights of all particles
	vector<double> weights;

public:
	// Set of current particles
	vector<Particle> particles;

	// Constructor
	// @param M Number of particles, whether the particle is initialized
	ParticleFilter() : num_particles(0), generator_(rd_()) {}

	// Destructor
	~ParticleFilter() {}

    bool isInitialized() const  { return initialized_; }

    Particle getMean() const { return mean_; }

	/**
	 * @brief Initializes particle filter by initializing particles to Gaussian
	 *        distribution around first position and all the weights set to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);


	/**
	 * @brief This function implements the PF init stage.
	*
	* @param num_particles Particle number in the filter.
	* @param x_init Init x-axis position.
	* @param y_init Init x-axis position.
	* @param z_init Init x-axis position.
	* @param a_init Init yaw angle orientation.
	* @param x_dev Init thresholds of x-axis position.
	* @param y_dev Init thresholds of y-axis position.
	* @param z_dev Init thresholds of z-axis position.
	* @param a_dev Init thresholds of yaw angle orientation.
	*
	* It restructures the particle vector to adapt it to the number of selected particles. Subsequently, it initializes
	* it using a Gaussian distribution and the deviation introduced. Subsequently, it calculates what would be the
	* average particle that would simulate the estimated position of the UAV.
   */
	void init(const int num_particles, const float x_init, const float y_init, const float z_init,
			  const float a_init, const float x_dev, const float y_dev, const float z_dev,
			  const float a_dev);

	/**
	 * @brief This function implements the PF prediction stage.
	* 		  (Translation in X, Y and Z in meters and yaw angle incremenet in rad.)
	*
	* @param odom_x_mod Increased odometry in the position of the x-axis.
	* @param odom_y_mod Increased odometry in the position of the x-axis.
	* @param odom_z_mod Increased odometry in the position of the x-axis.
	* @param odom_a_mod Increased odometry in the position of the x-axis.
	* @param delta_x Thresholds of x-axis position in prediction.
	* @param delta_y Thresholds of y-axis position in prediction.
	* @param delta_z Thresholds of z-axis position in prediction.
	* @param delta_a Thresholds of yaw angle orientation in prediction.
	*
	* It calculates the increase that has occurred in the odometry and makes predictions of where it is possible that the
	* UAV is, taking into account selected thresholds.
   */
	void predict(const double odom_x_mod, const double odom_y_mod, const double odom_z_mod,
				const double odom_a_mod, const double delta_x, const double delta_y, const double delta_z,
				const double delta_a);

	/** 
	 * @brief This function implements the PF update stage.
	*
	* @param grid3d Instance of the Grid3d class.
	* @param cloud Point cloud from the UAV view.
	* @param range_data Information of the radio-range sensor.
	* @param alpha Percentage weight between point cloud and range sensor.
	* @param sigma Desviation in the measurement of the radio-range sensor.
	*
	* It takes the positions of the particles to change if they are on the map. Then, it evaluates the weight of the
	* particle according to the point cloud and the measurement of the radio sensors. Finally, it normalizes the weights
	* for all particles and finds the average for the composition of the UAV pose.
   */
	void update(const Grid3d& grid3d, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
				const std::vector<Range>& range_data, const double alpha, const double sigma,
				const double roll, const double pitch);





	/**
	 * @brief Predicts the state for the next time step using the process model.
	 * @param delta_t: Time between time step t and t+1 in measurements [s]
	 * @param std_pos[]: Array of dimension 3 [standard deviation of x [m],
	 *																				 standard deviation of y [m],
	 *   																       standard deviation of yaw [rad]]
	 * @param velocity: Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate: Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[],
									double velocity, double yaw_rate);

	/**
	 * @brief Updates the weights for each particle based on the likelihood of the
	 * observed measurements.
	 * @param sensor_range: Range [m] of sensor
	 * @param std_landmark[]: Array of dimension 2 [standard deviation of range [m],
	 *   																						standard deviation of bearing [rad]]
	 * @param observations: Vector of landmark observations
	 * @param map: Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[],
										 vector<LandmarkObs> observations, Map map_landmarks);

	/**
	 * @brief Resample particles with replacement with probability proportional to weight
	 */
	void resample();

	void resample_basic();

	/**
	 * @brief Writes particle positions to a file.
	 * @param filename: File to write particle positions to.
	 */
	void write(string filename);


private:
    std::vector<Particle> p_; /*!< Vector of particles */
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

	/*
	 * Convert the passed in vehicle co-ordinates into map co-ordinates from
	 * the perspective of the particle in question
	 */
	 LandmarkObs convertVehicleToMapCoords(LandmarkObs observationToConvert,
 																				 Particle particle);
	 /*
 	 * Finds which observations correspond to which landmark
 	 * (likely by using a nearest-neighbors data association).
 	 * @param landmarks: List of landmarks
 	 * @param observation: Current list of converted observation
 	 */
	vector<LandmarkObs> dataAssociation(vector<Map::single_landmark_s> landmarks,
		 																	vector<LandmarkObs> observations);
};



#endif /* PARTICLE_FILTER_H_ */