#include <state_estimate_filter_ros/particle_filter.h>

using state_estimate_filter_ros::ParticleFilter;
using state_estimate_filter_ros::Particle;
using Eigen::MatrixXd;

// Class methods definitions
Particle::Particle() : state_(MatrixXd::Zero(0, 0)), weight_(MatrixXd::Zero(0, 0))
{
}

ParticleFilter::ParticleFilter() : StateEstimateFilter()
{
}

ParticleFilter::ParticleFilter(ros::NodeHandle& nh) : StateEstimateFilter(nh)
{
}

ParticleFilter::ParticleFilter(ros::NodeHandle& nh, const Eigen::MatrixXd system_a, const Eigen::MatrixXd system_b)
  : StateEstimateFilter(nh), system_a_(system_a), system_b_(system_b)
{
}

void ParticleFilter::predict(const Eigen::MatrixXd input_curr)
{
  predicted_val_ = system_a_ * input_curr + system_b_;
}

void ParticleFilter::filter(void)
{
}
