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
    return range_from + fmod((float)(int)(rand()), (range_to-range_from));
    //std::uniform_real_distribution<float> distribution(range_from, range_to);
    //return distribution(generator_);
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

void ParticleFilter::sampleParticlesGaussian(const float x_coord, const float y_coord,
                                            const float std_x, const float std_y, 
                                            const int num, vector<Particle> &out_particles)
{

    for (int i = 0; i < num; i++)
    {
        float rand_x = ranGaussian(x_coord, std_x);
        float rand_y = ranGaussian(y_coord, std_y);
        float rand_theta = ranGaussian(0, M_PI / 8.0);
        Particle ptc(rand_x, rand_y, rand_theta, 1.0); 
        out_particles.push_back(ptc);
    }
}

void ParticleFilter::set_params(
                   string map_file, string pcd_file, float resolution,
				   float fixed_height_, float lidar_to_wb_,
                   bool use_guess_, float init_x_, float init_y_,
                   int init_num_per_grid, int init_num_total, 
				   float _alpha1, float _alpha2, float _alpha3, float _alpha4,
				   float _sigma_hit, float _lambda_short, float _max_range, float _max_span,
				   float _z_hit, float _z_short, float _z_max, float _z_rand)
{
    fixed_height = fixed_height_; 
    lidar_to_wb = lidar_to_wb_;
    use_guess = use_guess_;
    init_x = init_x_;
    init_y = init_y_;
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


void ParticleFilter::init(int num_particles_total, bool use_guess, float coord_x, float coord_y)
{
    int particle_per_grid = 1;
    float total_to_free_cell_ratio = (float)num_particles_total / (float)grid_map.num_free_cells;
    float sample_area_width = grid_map.resolution;
    if (total_to_free_cell_ratio < 1) // if we don't want a particle in each cell
    {
        // then we expand the size of each "cell" that we sample from
        sample_area_width /= (float)total_to_free_cell_ratio;
    }
    //ROS_WARN_STREAM("sample area width: " << sample_area_width);
    particle_per_grid = std::max(1, std::min(num_particles_total, (int)total_to_free_cell_ratio));

    //ROS_WARN_STREAM("particle_per_grid: " << particle_per_grid);
    int num_particles = 0;
    // for debugging motion model:
    
    float start_x, start_y;
    // 204, 172, 1.57
    // 137, 347, 1.57?
    /*
    grid_map.idx_to_coord(204, 172, start_x, start_y);
    for (int i = 0; i < 100; i++)
    {
        Particle p_tmp(start_x, start_y, 1.57, 1.0);
        particles.push_back(p_tmp);
        num_particles++;
    }
    */
    
    // sample in free space (dubugging now)
    int tmp_rmin = 198;
    int tmp_rmax = 214;
    int tmp_cmin = 160;
    int tmp_cmax = 201;
    //particle_per_grid = std::max(1, std::min(100, 
     //           (int)((double)1000 / (double)((tmp_rmax - tmp_rmin)*(tmp_cmax-tmp_cmin)))));

    //ROS_WARN_STREAM("free cells: " << grid_map.num_free_cells << ", particle_per_grid: " << particle_per_grid);
    if (use_guess)
    {
        float sum_x = 0;
        float sum_y = 0;
        float sum_th = 0;
        int i = 0;
        while (i < num_particles_total)
        {
            float rand_x = ranGaussian(coord_x, 2.0);
            float rand_y = ranGaussian(coord_y, 2.0);
            float rand_theta = rngUniform(-M_PI, M_PI);
            Particle ptc(rand_x, rand_y, rand_theta, 1.0); 
            if (grid_map.isInMap(rand_x, rand_y))
            {
                particles.push_back(ptc);
                sum_x += rand_x;
                sum_y += rand_y;
                sum_th += rand_theta;
                i++;
            }
        }
        mean_ = Particle(sum_x/(float)num_particles_total, sum_y/(float)num_particles_total, warpAngle(sum_th/(float)num_particles_total), 1.0);
    }
    else
    {
        for (int r = 0; r < grid_map.rows; r++)
        {
            for (int c = 0; c < grid_map.cols; c++)
            {
                if (grid_map.occupancy_map[r][c] == FREE)
                {
                    pair<int, int> indices;
                    indices.first = r;
                    indices.second = c;
                    if (sample_area.find(indices) != sample_area.end())
                        continue;
                    float cx, cy, start_x, start_y, end_x, end_y;
                    grid_map.idx_to_coord(r, c, cx, cy); // center of the grid
                    start_x = cx - sample_area_width / 2;
                    start_y = cy - sample_area_width / 2;
                    end_x = start_x + sample_area_width;
                    end_y = start_y + sample_area_width;
                    sampleParticlesUniform(start_x, start_y, end_x, end_y, particle_per_grid, particles);
                    num_particles += particle_per_grid;
                    add_sample_area(sample_area_width, indices);
                }
            } 
        }  
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::uniform_int_distribution<int> idx_distrib(0, num_particles);
        int idx_rand = idx_distrib(gen);
        mean_ = particles[idx_rand]; // just a random particle from the set
    }
    initialized_ = true;
}

void ParticleFilter::add_sample_area(float sample_area_width, pair<int, int> indices)
{
    float coord_x, coord_y;
    grid_map.idx_to_coord(indices.first, indices.second, coord_x, coord_y);
    int idx_x_start, idx_y_start, idx_x_end, idx_y_end;
    grid_map.coord_to_idx(coord_x - sample_area_width/2, coord_y - sample_area_width/2, idx_x_start, idx_y_start);
    grid_map.coord_to_idx(coord_x + sample_area_width/2, coord_y + sample_area_width/2, idx_x_end, idx_y_end);
    for (int i = idx_x_start; i < idx_x_end+1; i++)
    {
        for (int j = idx_y_start; j < idx_y_end+1; j++)
        {
            pair<int, int> indices;
            indices.first = i;
            indices.second = j;
            if (grid_map.occupancy_map[i][j] == FREE && sample_area.find(indices) == sample_area.end())
            {
                sample_area.insert(indices);
            }
        }
    }
}

void ParticleFilter::predict(const Eigen::Vector3f &cur_odom, const Eigen::Vector3f &prev_odom)
{
    
    //const float x_dev = fabs(delta_x * odom_x_mod);
    //const float y_dev = fabs(delta_y * odom_y_mod);
    //const float theta_dev = fabs(delta_theta * odom_theta_mod);
    
    float delta_x = cur_odom[0] - prev_odom[0];
    float delta_y = cur_odom[1] - prev_odom[1];
    float delta_theta = cur_odom[2] - prev_odom[2];
    /*  Make a prediction for all particles according to the odometry */
    //float sa, ca, rand_x, rand_y;
    for (int i = 0; i < particles.size(); i++)
    {
        /*
        float d_trans = sqrt(delta_x * delta_x + delta_y * delta_y);
        float d_rot = atan2(delta_y, delta_x);
        particles[i].x += d_trans * cos(particles[i].theta + d_rot);
        particles[i].y += d_trans * sin(particles[i].theta + d_rot);
        particles[i].theta += delta_theta ;
        */

        // below is the original motion model
        
        float d_theta = warpAngle(delta_theta);

        float d_rot1 = atan2(delta_y, delta_x) - prev_odom[2];
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
    //ROS_WARN_STREAM("#particles: " << particles.size());
    /*  Incorporate measurements */
    float sum_weights = 0.0;

    vector<float> measurements;
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        Eigen::Vector3f p_meas(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
        measurements.push_back(p_meas.norm());
    }

    
    for (int i = 0; i < particles.size(); i++)
    {
        
        /*  Get particle information */
        float px = particles[i].x;
        float py = particles[i].y;
        float pth = particles[i].theta;

        /*  Check the particle is in the map */
        if (!grid_map.isInMap(px, py))
        {

            particles[i].weight = 0;
            continue;
        }

        /*  Evaluate the weight of the point cloud */
        particles[i].weight = computeCloudWeight(cloud_in, measurements, particles[i]);

        /*  Increase the summatory of weights */
        sum_weights += particles[i].weight;
    }

    /*  Normalize all weights */
    float wt = 0;
    float wt_max = 0;
    int idx_max = -1;
    vector<Particle> particlesCopy;
    bool bad_particles = false;
    for (int i = 0; i < particles.size(); i++)
    {
        if (sum_weights > 0)
        {
            particles[i].weight /= sum_weights;
            if (particles[i].weight != 0)
                particlesCopy.push_back(particles[i]);
        }
        else
        {
            particles[i].weight = 0;
            bad_particles = true;
            break;
        }
        if (idx_max == -1 || particles[i].weight > wt_max)
        {
            wt_max = particles[i].weight;
            idx_max = i;
        }
    }
    if (bad_particles || wt_max == 0)
    {
        ROS_WARN_STREAM("bad particles!!");
        particles.clear();
        init(100, true, mean_.x, mean_.y);
    }
    else
    {
        particles = particlesCopy;
        mean_ = particles[idx_max];
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
    int spawn = 30;
    std::vector<Particle> new_p(num_to_sample);
    const float factor = 1.f / (float)(num_to_sample+spawn);
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
    sampleParticlesGaussian(mean_.x, mean_.y, 0.2, 0.2, spawn, new_p);
    for (int i = num_to_sample; i < num_to_sample+spawn; i++)
        new_p[i].theta = warpAngle(new_p[i].theta + mean_.theta);
    particles = new_p;

    //! Asign the new particles set
}


float ParticleFilter::computeCloudWeight(LidarPointCloudConstPtr cloud, const vector<float> &measurements, Particle p)
{
    vector<float> dists; // computed distance from lidar to obstacle
    vector<float> angles;

    // transform cloud to particle frame
    LidarPointCloudPtr transformed_cloud(new LidarPointCloud);
    align_cloud_to_particle(cloud, p, transformed_cloud);

    Eigen::Vector3f robot_position(p.x, p.y, fixed_height);
    associate(transformed_cloud, point_cloud_map, kdtree, robot_position, dists, angles);

    float prob = 1.0;
    float sum_dists = 0;
    for (int i = 0; i < dists.size(); i++)
    {
        float meas = measurements[i];
        float meas_computed = dists[i];
        sum_dists += sqrt((meas - meas_computed)*(meas - meas_computed));
    }
    if (dists.size() != 0)
        // inverse average distance
        prob = 1.0 / (sum_dists / (float)dists.size());
    /*
    // compute weight
    for (int i = 0; i < dists.size(); i++)
    {
        float meas = measurements[i];
        float meas_computed = dists[i];
        float angle = fabs(warpAngle(angles[i]));
        float sig_hit_sqaure = sigma_hit * sigma_hit;
        float p_hit = (meas >= 0 && meas <= max_range) ? 
            (1.0 / sqrt(2*M_PI*sig_hit_sqaure))*exp(-0.5*((meas-meas_computed)*(meas-meas_computed)/sig_hit_sqaure)) : 0.0;
        float p_hit_angle = (angle >= 0 && angle <= 1.2) ? 
            (1.0 / sqrt(2*M_PI*0.25))*exp(-0.5*(angle*angle/0.25)) : 0.0;
        float p_short = (meas >= 0 && meas <= meas_computed) ?
            lambda_short * exp(-lambda_short * meas) : 0.0;
        float p_max = (meas > max_range-max_span && meas < max_range+max_span) ? 1.0/(2*max_span) : 0.0;
        float p_rand = (meas >= 0 && meas <= max_range) ? 1.0/max_range : 0.0;

        float p = z_hit * p_hit + z_short * p_short + z_max * p_max + z_rand * p_rand;

        if (p > 0) prob *= p;
    }
    //if (dists.size() != 0)
        //prob /= (float)dists.size();
    */
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

void ParticleFilter::buildParticleMsg(geometry_msgs::PoseStamped& msg) const
{
    msg.pose.position.x = static_cast<double>(mean_.x);
    msg.pose.position.y = static_cast<double>(mean_.y);
    msg.pose.position.z = static_cast<double>(fixed_height);
    msg.pose.orientation.x = 0.;
    msg.pose.orientation.y = 0.;
    msg.pose.orientation.z = sin(static_cast<double>(mean_.theta * 0.5f));
    msg.pose.orientation.w = cos(static_cast<double>(mean_.theta * 0.5f));
    
}

void ParticleFilter::align_cloud_to_particle(LidarPointCloudConstPtr cloud_meas_in, 
                                            const Particle p, 
                                            LidarPointCloudPtr cloud_aligned)
{
    float dx = lidar_to_wb * cos(p.theta);
    float dy = lidar_to_wb * sin(p.theta);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << p.x + dx, p.y + dy, fixed_height;
    // rotate around z axis
    transform.rotate(Eigen::AngleAxisf(p.theta, Eigen::Vector3f::UnitZ()));
    // Executing the transformation
    pcl::transformPointCloud(*cloud_meas_in, *cloud_aligned, transform);
}
