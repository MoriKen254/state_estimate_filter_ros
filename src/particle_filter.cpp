#include <state_estimate_filter_ros/particle_filter.h>

using state_estimate_filter_ros::ParticleFilter;

// Class methods definitions
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

void ParticleFilter::predict(const Eigen::MatrixXd vec_input_curr)
{
  // predicted_val = vec_input_curr + system_diff;
}

void ParticleFilter::filter(void)
{
}
