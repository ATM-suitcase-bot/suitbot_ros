#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

float ParticleFilter::ranGaussian(const double mean, const double sigma)
{
    std::normal_distribution<float> distribution(mean, sigma);
    return distribution(generator_);
}

float ParticleFilter::rngUniform(const float range_from, const float range_to)
{
    std::uniform_real_distribution<float> distribution(range_from, range_to);
    return distribution(generator_);
}

// Initializes particle filter by initializing particles to
// Gaussian distribution around first position and all the weights set to 1.
void ParticleFilter::init(double x, double y, double theta, double std[])
{
    // NOTE: The number of particles needs to be tuned
    num_particles = 200;

    // Object of random number engine class that generate pseudo-random numbers
    default_random_engine gen;

    // Standard deviations for x, y, and theta
    double std_x, std_y, std_theta;

    // Set standard deviations for x, y, and theta.
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];

    // Create a normal (Gaussian) distribution for position along x.
    normal_distribution<double> dist_x(x, std_x);
    // Create a normal (Gaussian) distribution for position along y.
    normal_distribution<double> dist_y(y, std_y);
    // Create a normal (Gaussian) distribution for heading of the car.
    normal_distribution<double> dist_theta(theta, std_theta);

    // Add random Gaussian noise to each particle.
    for (int par_index = 0; par_index < num_particles; ++par_index)
    {
        double sample_x, sample_y, sample_theta;

        // Sample from these normal distrubtions for x, y and theta
        // NOTE 1: "gen" is the random engine initialized earlier
        // NOTE 2: The generator object (g) supplies uniformly-distributed random
        //         integers through its operator() member function.
        //         The normal_distribution object transforms the values obtained
        //         this way so that successive calls to this member function with
        //         the same arguments produce floating-point values that follow a
        //         Normal distribution with the appropriate parameters.
        // NOTE 3: dist_x's constructor is overloaded with a member function of the
        //         same name.
        sample_x = dist_x(gen);
        sample_y = dist_y(gen);
        sample_theta = dist_theta(gen);

        // Variable of type Particle to store values before being appended to
        // the vector of particles
        Particle new_particle;

        // Set the id of the particle to be the same as the current index
        new_particle.id = par_index;
        // Set the particle position in x, y and angle theta from the individual
        // samples from the distribution of the respective means and sigmas
        new_particle.x = sample_x;
        new_particle.y = sample_y;
        new_particle.theta = sample_theta;

        // The weight needs to be set to 1.0 initially
        new_particle.weight = 1.0;

        // Append the new particle to the vector
        particles.push_back(new_particle);
    }

    // Since this function is called only once(first measurement), set to True
    initialized_ = true;
}

// from amcl3d
void ParticleFilter::init(const int num_particles, const float x_init, const float y_init, const float z_init,
                          const float a_init, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev)
{
    /*  Resize particle set */
    p_.resize(abs(num_particles));

    /*  Sample the given pose */
    const float dev = std::max(std::max(x_dev, y_dev), z_dev);
    const float gauss_const_1 = 1. / (dev * sqrt(2 * M_PI));
    const float gauss_const_2 = 1. / (2 * dev * dev);

    p_[0].x = x_init;
    p_[0].y = y_init;
    p_[0].z = z_init;
    p_[0].a = a_init;
    p_[0].w = gauss_const_1;

    float wt = p_[0].w;
    float dist;

    for (uint32_t i = 1; i < p_.size(); ++i)
    {
        p_[i].x = p_[0].x + ranGaussian(0, x_dev);
        p_[i].y = p_[0].y + ranGaussian(0, y_dev);
        p_[i].z = p_[0].z + ranGaussian(0, z_dev);
        p_[i].a = p_[0].a + ranGaussian(0, a_dev);

        dist = sqrt((p_[i].x - p_[0].x) * (p_[i].x - p_[0].x) + (p_[i].y - p_[0].y) * (p_[i].y - p_[0].y) +
                    (p_[i].z - p_[0].z) * (p_[i].z - p_[0].z));

        p_[i].w = gauss_const_1 * exp(-dist * dist * gauss_const_2);

        wt += p_[i].w;
    }

    Particle mean_p;
    for (uint32_t i = 0; i < p_.size(); ++i)
    {
        p_[i].w /= wt;

        mean_p.x += p_[i].w * p_[i].x;
        mean_p.y += p_[i].w * p_[i].y;
        mean_p.z += p_[i].w * p_[i].z;
        mean_p.a += p_[i].w * p_[i].a;
    }
    mean_ = mean_p;

    initialized_ = true;
}


void ParticleFilter::predict(const double odom_x_mod, const double odom_y_mod, const double odom_z_mod,
                             const double odom_a_mod, const double delta_x, const double delta_y, const double delta_z,
                             const double delta_a)
{
  const double x_dev = fabs(delta_x * odom_x_mod);
  const double y_dev = fabs(delta_y * odom_y_mod);
  const double z_dev = fabs(delta_z * odom_z_mod);
  const double a_dev = fabs(delta_a * odom_a_mod);

  /*  Make a prediction for all particles according to the odometry */
  float sa, ca, rand_x, rand_y;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    sa = sin(p_[i].a);
    ca = cos(p_[i].a);
    rand_x = delta_x + ranGaussian(0, x_dev);
    rand_y = delta_y + ranGaussian(0, y_dev);
    p_[i].x += ca * rand_x - sa * rand_y;
    p_[i].y += sa * rand_x + ca * rand_y;
    p_[i].z += delta_z + ranGaussian(0, z_dev);
    p_[i].a += delta_a + ranGaussian(0, a_dev);
  }
}

void ParticleFilter::update(const Grid3d& grid3d, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            const std::vector<Range>& range_data, const double alpha, const double sigma,
                            const double roll, const double pitch)
{
  /*  Incorporate measurements */
  float wtp = 0, wtr = 0;

  clock_t begin_for1 = clock();
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    /*  Get particle information */
    float tx = p_[i].x;
    float ty = p_[i].y;
    float tz = p_[i].z;

    /*  Check the particle is into the map */
    if (!grid3d.isIntoMap(tx, ty, tz))
    {
      // std::cout << "Not into map: " << grid3d_.isIntoMap(tx, ty, tz-1.0) << std::endl;
      p_[i].w = 0;
      continue;
    }

    /*  Evaluate the weight of the point cloud */
    p_[i].wp = grid3d.computeCloudWeight(cloud, tx, ty, tz, roll, pitch, p_[i].a);

    /*  Evaluate the weight of the range sensors */
    p_[i].wr = computeRangeWeight(tx, ty, tz, range_data, sigma);

    /*  Increase the summatory of weights */
    wtp += p_[i].wp;
    wtr += p_[i].wr;
  }
  clock_t end_for1 = clock();
  double elapsed_secs = double(end_for1 - begin_for1) / CLOCKS_PER_SEC;
  ROS_DEBUG("Update time 1: [%lf] sec", elapsed_secs);

  /*  Normalize all weights */
  float wt = 0;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if (wtp > 0)
      p_[i].wp /= wtp;
    else
      p_[i].wp = 0;

    if (wtr > 0)
      p_[i].wr /= wtr;
    else
      p_[i].wr = 0;

    if (!grid3d.isIntoMap(p_[i].x, p_[i].y, p_[i].z))
    {
      /* std::cout << "Not into map: " << grid3d_.isIntoMap(tx, ty, tz-1.0) << std::endl; */
      p_[i].w = 0;
    }
    else
      p_[i].w = p_[i].wp * alpha + p_[i].wr * (1 - alpha);
    wt += p_[i].w;
  }

  Particle mean_p;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if (wt > 0)
      p_[i].w /= wt;
    else
      p_[i].w = 0;

    mean_p.x += p_[i].w * p_[i].x;
    mean_p.y += p_[i].w * p_[i].y;
    mean_p.z += p_[i].w * p_[i].z;
    mean_p.a += p_[i].w * p_[i].a;
  }
  mean_ = mean_p;
}

void ParticleFilter::resample()
{
  std::vector<Particle> new_p(p_.size());
  const float factor = 1.f / p_.size();
  const float r = factor * rngUniform(0, 1);
  float c = p_[0].w;
  float u;

  //! Do resamplig
  for (uint32_t m = 0, i = 0; m < p_.size(); ++m)
  {
    u = r + factor * m;
    while (u > c)
    {
      if (++i >= p_.size())
        break;
      c += p_[i].w;
    }
    new_p[m] = p_[i];
    new_p[m].w = factor;
  }

  //! Asign the new particles set
  p_ = new_p;
}






// Predicts the state(set of particles) for the next time step
// using the process model.
void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
    // Add measurements to each particle and add random Gaussian noise.
    // Object of random number engine class that generate pseudo-random numbers
    default_random_engine gen;

    // Standard deviations for x, y, and theta
    double std_x, std_y, std_theta;

    // Set standard deviations for x, y, and theta.
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];

    // Create a normal (Gaussian) distribution for noise along position x.
    normal_distribution<double> noise_dist_x(0, std_x);
    // Create a normal (Gaussian) distribution for noise along position y.
    normal_distribution<double> noise_dist_y(0, std_y);
    // Create a normal (Gaussian) distribution for noise of direction theta.
    normal_distribution<double> noise_dist_theta(0, std_theta);

    // NOTE: Adding noise using the following resources
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    // Prediction for position x,y and angle theta for each of the particles
    for (size_t par_index = 0; par_index < particles.size(); par_index++)
    {
        // Temporary variable to store the particle's previous state's theta
        double prev_theta = particles[par_index].theta;

        // Avoid divide by zero error and update prediction for the particle
        if (abs(yaw_rate) > 0.0001)
        {
            // Update the position x, y and angle theta of the particle
            particles[par_index].x += (velocity / yaw_rate) *
                                      (sin(prev_theta + (yaw_rate * delta_t)) -
                                       sin(prev_theta));
            particles[par_index].y += (velocity / yaw_rate) *
                                      (cos(prev_theta) -
                                       cos(prev_theta + (yaw_rate * delta_t)));
        }
        else
        {
            // Update the position x, y and angle theta of the particle
            particles[par_index].x += velocity * delta_t * cos(prev_theta);
            particles[par_index].y += velocity * delta_t * sin(prev_theta);
        }
        // Update theta
        particles[par_index].theta = prev_theta + yaw_rate * delta_t;

        // Add random gaussian noise for each of the above updated measurements
        particles[par_index].x += noise_dist_x(gen);
        particles[par_index].y += noise_dist_y(gen);
        particles[par_index].theta += noise_dist_theta(gen);
    }
}

// Find the closest landmark to the current observation
vector<LandmarkObs> ParticleFilter::dataAssociation(vector<Map::single_landmark_s> landmarks,
                                                    vector<LandmarkObs> observations)
{
    // Vector of associated landmarks
    vector<LandmarkObs> associatedLandmarks;

    // Go through list of observations
    for (size_t obs_index = 0; obs_index < observations.size(); obs_index++)
    {
        // Start of with the maximum possible value
        double minDistance = DBL_MAX;
        size_t indexOfLandmark;

        // Find the landmark closest to the observation
        for (size_t land_index = 0; land_index < landmarks.size(); land_index++)
        {
            double currentDistance = dist(landmarks[land_index].x_f,
                                          landmarks[land_index].y_f,
                                          observations[obs_index].x,
                                          observations[obs_index].y);

            // Update the minimum distance found and the index if
            // another landmark is closer to this observation
            if (currentDistance <= minDistance)
            {
                minDistance = currentDistance;
                indexOfLandmark = land_index;
            }
        }

        LandmarkObs closestLandmark;
        closestLandmark.id = landmarks[indexOfLandmark].id_i;
        closestLandmark.x = landmarks[indexOfLandmark].x_f;
        closestLandmark.y = landmarks[indexOfLandmark].y_f;
        associatedLandmarks.push_back(closestLandmark);
    }

    // Return the associated landmarks
    return associatedLandmarks;
}

// Update all the weights of the particles in the particle filter
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   vector<LandmarkObs> observations,
                                   Map map_landmarks)
{
    // Set standard deviations for x, y
    double std_x, std_y;
    double var_x, var_y;

    // Set the standard deviation and calculate variance
    std_x = std_landmark[0];
    std_y = std_landmark[1];
    var_x = std_x * std_x;
    var_y = std_y * std_y;

    // Go through the list of particles
    for (size_t par_index = 0; par_index < particles.size(); par_index++)
    {
        // For the given list of landmarks find the predicted landmarks within
        // the range of the car sensor
        vector<Map::single_landmark_s> predicted_landmarks;
        for (size_t land_index = 0; land_index < map_landmarks.landmark_list.size(); land_index++)
        {
            // Calculate the difference between the particle prediction & landmark
            double distanceDiff = dist(particles[par_index].x,
                                       particles[par_index].y,
                                       map_landmarks.landmark_list[land_index].x_f,
                                       map_landmarks.landmark_list[land_index].y_f);

            // Create a new list of landmarks within sensor range for data association
            if (distanceDiff <= sensor_range)
            {
                predicted_landmarks.push_back(map_landmarks.landmark_list[land_index]);
            }
        }

        // Vector for converted observations
        vector<LandmarkObs> convertedObservations;

        // For the list of observations, convert to map-coordinates, find the
        // closest landmark and finally update the weight of the particle
        for (size_t obs_index = 0; obs_index < observations.size(); obs_index++)
        {
            // Convert from car to map-coordinates
            LandmarkObs convertedObs = convertVehicleToMapCoords(observations[obs_index],
                                                                 particles[par_index]);

            // Push to the new list of converted observations
            convertedObservations.push_back(convertedObs);
        }

        // Using the converted observations perform data association
        vector<LandmarkObs> associatedLandmarks = dataAssociation(predicted_landmarks,
                                                                  convertedObservations);

        // Variable to store the result of the multivariate-gaussian
        double multi_gaussian = 1.0;

        // Update weight of the particle
        for (size_t obs_index = 0; obs_index < associatedLandmarks.size(); obs_index++)
        {
            // Update the weights of each particle using a
            // a multi-variate Gaussian distribution.
            // Info: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
            // Set standard deviations for x, y
            double std_x, std_y;
            double var_x, var_y;

            // Set the standard deviation and calculate variance
            std_x = std_landmark[0];
            std_y = std_landmark[1];
            var_x = std_x * std_x;
            var_y = std_y * std_y;

            // Variables to store the square of the difference between measured and
            // predicted x and y values
            double sq_diff_x = associatedLandmarks[obs_index].x -
                               convertedObservations[obs_index].x;
            double sq_diff_y = associatedLandmarks[obs_index].y -
                               convertedObservations[obs_index].y;

            sq_diff_x = sq_diff_x * sq_diff_x;
            sq_diff_y = sq_diff_y * sq_diff_y;

            multi_gaussian *= (1 / (2 * M_PI * std_x * std_y)) *
                              exp(-((sq_diff_x) / (2 * var_x) + (sq_diff_y) / (2 * var_y)));
        }

        // Update the weight of the particle
        particles[par_index].weight = multi_gaussian;
    }
}

// Resample particles with replacement with probability proportional to weight.
void ParticleFilter::resample_basic()
{
    // Copy of the particles vector list
    vector<Particle> particlesCopy = particles;

    // Empty the existing particle list
    particles.erase(particles.begin(), particles.end());

    // Vector of weights of the particles
    vector<double> weights;
    for (size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
    {
        weights.push_back(particlesCopy[par_index].weight);
    }

    // Object of random number engine class that generate pseudo-random numbers
    // NOTE: http://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine
    mt19937 gen;

    // Object for generating discrete distribution based on the weights vector
    discrete_distribution<double> weights_dist(weights.begin(), weights.end());

    // With the discrete distribution pick out particles according to their
    // weights. The higher the weight of the particle, the higher are the chances
    // of the particle being included multiple times.
    // Discrete_distribution is used here to pick particles with the appropriate
    // weights(i.e. which meet a threshold)
    // http://www.cplusplus.com/reference/random/discrete_distribution/
    // NOTE: Here is an example which helps with the understanding
    //       http://coliru.stacked-crooked.com/a/3c9005a4cc0ed9d6
    for (size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
    {
        // Append the particle to the new list
        // NOTE: Calling weights_dist with the generator returns the index of one
        //       of weights in the vector which was used to generate the distribution.
        particles.push_back(particlesCopy[weights_dist(gen)]);
    }
}

// Writes particle positions to a file.
void ParticleFilter::write(string filename)
{
    // Object of ofstream for writing output data
    ofstream dataFile;

    // Delete existing files because we always write at the end of the file
    remove(filename.c_str());

    // Open the file with the filename passed in and with ios::app option set
    // NOTE: When ios::app is set, all output operations are performed at the end
    //       of the file.
    dataFile.open(filename, ios::app);

    // Go through each particle and write the particle data into the file
    for (int par_index = 0; par_index < num_particles; ++par_index)
    {
        if (par_index == num_particles - 1)
        {
            dataFile << particles[par_index].x << ","
                     << particles[par_index].y << ","
                     << particles[par_index].theta;
        }
        else
        {
            dataFile << particles[par_index].x << ","
                     << particles[par_index].y << ","
                     << particles[par_index].theta << "\n";
        }
    }

    // Close the file
    dataFile.close();
}

// Convert the passed in vehicle co-ordinates into map co-ordinates from
// the perspective of the particle in question
LandmarkObs ParticleFilter::convertVehicleToMapCoords(LandmarkObs observationToConvert,
                                                      Particle particle)
{
    // NOTE: The observations are given in the VEHICLE'S coordinate system.
    // 	     Your particles are located according to the MAP'S coordinate system.
    //       A transformation is required between the two systems.
    //   		 The following is a good resource for the theory:
    //   		 https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //  		 and the following is a good resource for the actual equation to
    //       implement (look at equation 3.33. The equation stays as it is.
    //       1. http://planning.cs.uiuc.edu/node99.html
    //       2. http://www.sunshine2k.de/articles/RotationDerivation.pdf
    LandmarkObs convertedObservation;
    convertedObservation.id = observationToConvert.id;
    convertedObservation.x = particle.x +
                             observationToConvert.x * cos(particle.theta) -
                             observationToConvert.y * sin(particle.theta);

    convertedObservation.y = particle.y +
                             observationToConvert.x * sin(particle.theta) +
                             observationToConvert.y * cos(particle.theta);

    return convertedObservation;
}