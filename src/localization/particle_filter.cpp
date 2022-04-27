#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"


ParticleFilter::ParticleFilter(string map_file, string pcd_file) : initial_num_particles_per_grid(50), 
                                                                    initial_num_particles_total(10000), 
                                                                    point_cloud_map(new LidarPointCloud)
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


ParticleFilter::ParticleFilter(string map_file, string pcd_file, 
                                float fixed_height_, int init_num_per_grid, int init_num_total) : 
                                fixed_height(fixed_height_), 
                                initial_num_particles_per_grid(init_num_per_grid), 
                                initial_num_particles_total(init_num_total), 
                                point_cloud_map(new LidarPointCloud)
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



ParticleFilter::ParticleFilter(string map_file, string pcd_file, 
				   float fixed_height_, int init_num_per_grid, int init_num_total, 
				   float _alpha1, float _alpha2, float _alpha3, float _alpha4,
				   float _sigma_hit, float _lambda_short, float _max_range, float _max_span,
				   float _z_hit, float _z_short, float _z_max, float _z_rand) : 
				   fixed_height(fixed_height_), 
				   initial_num_particles_per_grid(init_num_per_grid), 
				   initial_num_particles_total(init_num_total), 
				   alpha1(_alpha1), alpha2(_alpha2), alpha3(_alpha3), alpha4(_alpha4), 
				   sigma_hit(_sigma_hit), lambda_short(_lambda_short), max_range(_max_range), max_span(_max_span),
				   z_hit(_z_hit), z_short(_z_short), z_max(_z_max), z_rand(_z_rand),
				   point_cloud_map(new LidarPointCloud)
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


float ParticleFilter::ranGaussian(const float mean, const float sigma)
{

    std::normal_distribution<float> distribution(mean, sigma);
    return distribution(generator_);
}

float ParticleFilter::rngUniform(const float range_from, const float range_to)
{
    std::uniform_real_distribution<float> distribution(range_from, range_to);
    return distribution(generator_);
}

void ParticleFilter::sampleParticlesUniform(const float from_x, const float from_y,
                                            const float to_x, const float to_y,
                                            const int num, vector<Particle> &out_particles)
{

    for (int i = 0; i < num; i++)
    {
        float rand_x = rngUniform(from_x, to_x);
        float rand_y = rngUniform(from_y, to_y);
        float rand_theta = rngUniform(-M_PI, M_PI);
        Particle ptc(rand_x, rand_y, rand_theta, 1.0); 
        out_particles.push_back(ptc);
    }
}

void ParticleFilter::set_params(
                   string map_file, string pcd_file, float resolution,
				   float fixed_height_, int init_num_per_grid, int init_num_total, 
				   float _alpha1, float _alpha2, float _alpha3, float _alpha4,
				   float _sigma_hit, float _lambda_short, float _max_range, float _max_span,
				   float _z_hit, float _z_short, float _z_max, float _z_rand)
{
    fixed_height = fixed_height_; 
    initial_num_particles_per_grid = init_num_per_grid; 
    initial_num_particles_total = init_num_total;
    int res = grid_map.initOccupancyGridMap(map_file, resolution);
    if (res < 0)
    {
        std::cerr << "ERROR: Cannot create occupancy map! Exiting." << endl;
        exit(1);
    }
    loadPCDMap(pcd_file, point_cloud_map);
    kdtree.setInputCloud(point_cloud_map);
    alpha1 = _alpha1;
    alpha2 = _alpha2;
    alpha3 = _alpha3;
    alpha4 = _alpha4;
    sigma_hit = _sigma_hit;
    lambda_short = _lambda_short;
    max_range = _max_range;
    max_span = _max_span;
    z_hit = _z_hit;
    z_short = _z_short;
    z_max = _z_max;
    z_rand = _z_rand;
}


void ParticleFilter::init()
{
    int particle_per_grid = std::max(1, std::min(initial_num_particles_per_grid, 
                (int)((double)initial_num_particles_total / (double)grid_map.num_free_cells)));

    ROS_WARN_STREAM("particle_per_grid: " << particle_per_grid);
    int num_particles = 0;

    // sample in free space
    for (int r = 0; r < grid_map.rows; r++)
    {
        for (int c = 0; c < grid_map.cols; c++)
        {
            if (grid_map.occupancy_map[r][c] == FREE)
            {
                float start_x, start_y, end_x, end_y;
                grid_map.idx_to_coord(r, c, start_x, start_y); // center of the grid
                start_x -= grid_map.resolution;
                start_y -= grid_map.resolution;
                end_x = start_x + grid_map.resolution * 2;
                end_y = start_y + grid_map.resolution * 2;
                sampleParticlesUniform(start_x, start_y, end_x, end_y, particle_per_grid, particles);
                num_particles += particle_per_grid;
            }
        } 
    }
    std::uniform_int_distribution<int> idx_distrib(0, num_particles);
    int idx_rand = idx_distrib(generator_);
    mean_ = particles[idx_rand]; // just a random particle from the set

    initialized_ = true;
}



void ParticleFilter::predict(const float delta_x, const float delta_y, const float delta_theta)
{
    /*
    const float x_dev = fabs(delta_x * odom_x_mod);
    const float y_dev = fabs(delta_y * odom_y_mod);
    const float theta_dev = fabs(delta_theta * odom_theta_mod);
    */

    /*  Make a prediction for all particles according to the odometry */
    //float sa, ca, rand_x, rand_y;
    for (int i = 0; i < particles.size(); i++)
    {
        /*
        sa = sin(particles[i].theta);
        ca = cos(particles[i].theta);
        rand_x = delta_x + ranGaussian(0, x_dev);
        rand_y = delta_y + ranGaussian(0, y_dev);
        particles[i].x += ca * rand_x - sa * rand_y;
        particles[i].y += sa * rand_x + ca * rand_y;
        particles[i].theta += delta_theta + ranGaussian(0, theta_dev);
        */
        float d_theta = warpAngle(delta_theta);

        float d_rot1 = atan2(delta_y, delta_x) - particles[i].theta;
        float d_trans = sqrt(delta_x * delta_x + delta_y * delta_y);
        float d_rot2 = d_theta - d_rot1;

        d_rot1 = warpAngle(d_rot1);
        d_rot2 = warpAngle(d_rot2);

        float hd_rot1 = d_rot1 - ranGaussian(0.0, sqrt(alpha1 * d_rot1*d_rot1 + alpha2 * d_trans*d_trans));
        float hd_trans = d_trans - ranGaussian(0.0, sqrt(alpha3 * d_trans*d_trans + alpha4 * (d_rot1*d_rot1 + d_rot2*d_rot2)));
        float hd_rot2 = d_rot2 - ranGaussian(0.0, sqrt(alpha1 * d_rot2*d_rot2 + alpha2 * d_trans*d_trans));

        // update partical coord
        particles[i].x = particles[i].x + hd_trans * cos(particles[i].theta + hd_rot1);
        particles[i].y = particles[i].y + hd_trans * sin(particles[i].theta + hd_rot1);
        particles[i].theta = warpAngle(particles[i].theta + hd_rot1 + hd_rot2);
    }
}


// based on p2p distance
void ParticleFilter::update(LidarPointCloudConstPtr cloud_in)
{
    /*  Incorporate measurements */
    float sum_weights = 0.0;

    vector<float> measurements;
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        Eigen::Vector3f p_meas(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
        measurements.push_back(p_meas.norm());
    }

    TicToc tic_toc;
    for (int i = 0; i < particles.size(); i++)
    {
        /*  Get particle information */
        float px = particles[i].x;
        float py = particles[i].y;
        float pth = particles[i].theta;

        /*  Check the particle is in the map */
        if (grid_map.isInMap(px, py))
        {
            particles[i].weight = 0;
            continue;
        }

        /*  Evaluate the weight of the point cloud */
        particles[i].weight = computeCloudWeight(cloud_in, measurements, px, py, pth);

        /*  Increase the summatory of weights */
        sum_weights += particles[i].weight;
    }
    cout << "Update time: " <<  tic_toc.toc() << "ms" << endl;

    /*  Normalize all weights */
    float wt = 0;
    for (int i = 0; i < particles.size(); i++)
    {
        if (sum_weights > 0)
            particles[i].weight /= sum_weights;
        else
            particles[i].weight = 0;
    }
/*
    // instead of this, we do non maximum supression
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
    */
}

void ParticleFilter::resample(int num_to_sample)
{
  std::vector<Particle> new_p(num_to_sample);
  const float factor = 1.f / num_to_sample;
  const float r = factor * rngUniform(0, 1);
  float c = particles[0].weight;
  float u;

  //! Do resamplig
  for (int m = 0, i = 0; m < num_to_sample; m++)
  {
    u = r + factor * m;
    while (u > c)
    {
      if (++i >= particles.size())
        break;
      c += particles[i].weight;
    }
    new_p[m] = particles[i];
    new_p[m].weight = factor;
  }

  //! Asign the new particles set
  particles = new_p;
}


float ParticleFilter::computeCloudWeight(LidarPointCloudConstPtr cloud, const vector<float> &measurements, const float px, const float py, const float pth)
{
    vector<float> dists; // computed distance from lidar to obstacle
    vector<float> angles;

    // transform cloud to particle frame
    // TODO check if we need to rotate z to point towards the ground
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << px, py, fixed_height;
    // rotate around z axis
    float theta = pth;
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    // Executing the transformation
    LidarPointCloudPtr transformed_cloud(new LidarPointCloud);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    Eigen::Vector3f robot_position(px, py, fixed_height);
    associate(transformed_cloud, point_cloud_map, kdtree, robot_position, dists, angles);

    // compute weight
    float prob = 1.0;
    for (int i = 0; i < dists.size(); i++)
    {
        float meas = measurements[i];
        float meas_computed = dists[i];
        float sig_hit_sqaure = sigma_hit * sigma_hit;
        float p_hit = (meas >= 0 && meas <= max_range) ? 
            (1.0 / sqrt(2*M_PI*sig_hit_sqaure))*exp(-0.5*((meas-meas_computed)*(meas-meas_computed)/sig_hit_sqaure)) : 0.0;
        float p_short = (meas >= 0 && meas <= meas_computed) ?
            lambda_short * exp(-lambda_short * meas) : 0.0;
        float p_max = (meas > max_range-max_span && meas < max_range+max_span) ? 1.0/(2*max_span) : 0.0;
        float p_rand = (meas >= 0 && meas <= max_range) ? 1.0/max_range : 0.0;

        float p = z_hit * p_hit + z_short * p_short + z_max * p_max + z_rand * p_rand;

        if (p > 0) prob *= p;
    }
    return prob;
}






// Resample particles with replacement with probability proportional to weight.
void ParticleFilter::resample_basic()
{
    vector<Particle> particlesCopy = particles;
    particles.erase(particles.begin(), particles.end());

    vector<float> weights;
    for (size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
    {
        weights.push_back(particlesCopy[par_index].weight);
    }

    discrete_distribution<int> weights_dist(weights.begin(), weights.end());

    for (size_t par_index = 0; par_index < particlesCopy.size(); par_index++)
    {
        particles.push_back(particlesCopy[weights_dist(generator_)]);
    }
}

// Writes particle positions to a file.
void ParticleFilter::write(string filename)
{
    ofstream dataFile;

    remove(filename.c_str());

    // Open the file with the filename passed in and with ios::app option set
    // NOTE: When ios::app is set, all output operations are performed at the end
    //       of the file.
    dataFile.open(filename, ios::app);
/*
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
*/
    // Close the file
    dataFile.close();
}

void ParticleFilter::buildParticlesPoseMsg(geometry_msgs::PoseArray& msg) const
{
    msg.poses.resize(particles.size());

    for (int i = 0; i < particles.size(); i++)
    {
        msg.poses[i].position.x = static_cast<double>(particles[i].x);
        msg.poses[i].position.y = static_cast<double>(particles[i].y);
        msg.poses[i].position.z = static_cast<double>(fixed_height);
        msg.poses[i].orientation.x = 0.;
        msg.poses[i].orientation.y = 0.;
        msg.poses[i].orientation.z = sin(static_cast<double>(particles[i].theta * 0.5f));
        msg.poses[i].orientation.w = cos(static_cast<double>(particles[i].theta * 0.5f));
    }
}