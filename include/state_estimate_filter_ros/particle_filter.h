#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "state_estimate_filter.h"
#include <probability_distribution/normal_distribution.h>

namespace state_estimate_filter_ros
{
class Particle
{
public:
  Particle();
  Particle(const Eigen::MatrixXd state, const Eigen::MatrixXd weight);

public: // instead of property ...
  Eigen::MatrixXd state_;
  Eigen::MatrixXd weight_;
};

class ParticleFilter : public StateEstimateFilter
{
public:  // constructors & destoructors
  ParticleFilter();
  ParticleFilter(ros::NodeHandle& nh);
  ParticleFilter(ros::NodeHandle& nh,
                 const Eigen::MatrixXd system_a, const Eigen::MatrixXd system_b, const Eigen::MatrixXd system_c,
                 const int num_particle, const Eigen::MatrixXd state, const Eigen::MatrixXd weight);

public:                                                // methods
  void predict(const Eigen::MatrixXd input_curr);      // 予測ステップ(事前推定)
  void filter();                                       // フィルタリングステップ(事後推定)

private:
  void initParicles(const int num, const Eigen::MatrixXd state, const Eigen::MatrixXd weight);

private:
  Eigen::MatrixXd system_a_;
  Eigen::MatrixXd system_b_;
  Eigen::MatrixXd system_c_;
  std::vector<Particle> particles_;
  std::default_random_engine gen_system_;
  std::normal_distribution<double> dist_system_;
  probability_distribution::ProbabilityDistribution* prob_dis_;
};
}

#endif  // PARTICLE_FILTER_H
