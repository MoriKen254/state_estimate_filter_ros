#include <state_estimate_filter_ros/particle_filter.h>

using dist_type = std::normal_distribution<>;
using Eigen::MatrixXd;

using state_estimate_filter_ros::Particle;
using state_estimate_filter_ros::ParticleFilter;
using probability_distribution::NormalDistribution;
using probability_distribution::ProbabilityDistribution;

// Class methods definitions
Particle::Particle() : state_(MatrixXd::Zero(1, 1)), weight_(MatrixXd::Zero(1, 1)), logDencity_(MatrixXd::Zero(1, 1))
{
}

Particle::Particle(MatrixXd state, MatrixXd weight)
  : state_(state), weight_(weight)
{
  logDencity_ = MatrixXd::Zero(1, 1);
}

ParticleFilter::ParticleFilter() : StateEstimateFilter()
{
}

ParticleFilter::ParticleFilter(ros::NodeHandle& nh) : StateEstimateFilter(nh)
{
}

ParticleFilter::ParticleFilter(ros::NodeHandle& nh,
                               const MatrixXd system_a, const MatrixXd system_b, const MatrixXd system_c,
                               const int num_particle, const MatrixXd state, const MatrixXd weight)
  : StateEstimateFilter(nh), system_a_(system_a), system_b_(system_b), system_c_(system_c)
{
  gen_system_.seed();
  dist_type::param_type param(0.0, 1.0);
  dist_system_.param(param);

  initParicles(num_particle, state, weight);
}

void ParticleFilter::initParicles(const int num, const MatrixXd state, const MatrixXd weight)
{
  for(int i = 0; i < num; i++)
  {
    particles_.push_back(Particle(state, weight));
  }
}

void ParticleFilter::predict(const MatrixXd vec_input_curr)
{
  MatrixXd noise(1, 1);

  std::vector<Particle>::iterator itr = particles_.begin();

  // int i = 0;
  double sum = 0;
  for(; itr != particles_.end(); ++itr)
  {
    noise(0, 0) = dist_system_(gen_system_);
    // std::cout << "prev: particle[" << i << "] ..." << itr->state_ << std::endl;
    itr->state_ = system_a_ * itr->state_ + system_b_ * vec_input_curr + system_c_ + noise;
    // std::cout << "curr: particle[" << i << "] ..." << itr->state_ << std::endl;
    sum += itr->state_(0, 0);
  }
  vec_predict_curr_(0, 0) = sum / static_cast<double>(particles_.size());
  //vec_predict_curr_ = system_a_ * itr->state_ + system_b_ * input_curr + system_c_ + noise;
}

void ParticleFilter::filter(const MatrixXd vec_observation_curr)
{
  MatrixXd variance(1, 1);
  variance(0, 0) = 5*5;

  ProbabilityDistribution* prob_dis = new NormalDistribution(vec_observation_curr, variance);

  MatrixXd log_dencity_max(1, 1);
  log_dencity_max(0, 0) = 0.0;

  std::vector<Particle>::iterator itr;
  for(itr = particles_.begin(); itr != particles_.end(); ++itr)
  {
    itr->logDencity_ = prob_dis->calcLogDencityFunction(itr->state_);
    if (itr->logDencity_(0, 0) > log_dencity_max(0, 0))
      log_dencity_max = itr->logDencity_;
  }

  MatrixXd log_dencity_sum(1, 1);
  log_dencity_sum(0, 0) = 0.0;
  for(itr = particles_.begin(); itr != particles_.end(); ++itr)
  {
    itr->weight_ = itr->logDencity_ - log_dencity_max;
    log_dencity_sum += itr->weight_;
  }

  // normalize
  for(itr = particles_.begin(); itr != particles_.end(); ++itr)
  {
    itr->weight_(0, 0) = itr->weight_(0, 0) / log_dencity_sum(0, 0);
  }

  /*
  // just for check
  MatrixXd weight_sum(1, 1);
  weight_sum(0, 0) = 0.0;
  for(itr = particles_.begin(); itr != particles_.end(); ++itr)
  {
    weight_sum += itr->weight_;
  }
  */

  // estimation
  vec_estimate_curr_(0, 0) = 0.0;
  for(itr = particles_.begin(); itr != particles_.end(); ++itr)
  {
    vec_estimate_curr_ = vec_estimate_curr_ + itr->weight_ * itr->state_;
  }

  //vec_estimate_curr_ = vec_predict_curr_;
  vec_estimate_prev_ = vec_estimate_curr_;
  vec_predict_prev_ = vec_predict_curr_;

}
