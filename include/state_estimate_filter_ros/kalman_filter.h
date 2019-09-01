#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "state_estimate_filter.h"
#include <probability_distribution/normal_distribution.h>

namespace state_estimate_filter_ros
{
class Particle2
{
public:
  Particle2();
  Particle2(const Eigen::MatrixXd state, const Eigen::MatrixXd weight);

public: // instead of property ...
  Eigen::MatrixXd state_;
  Eigen::MatrixXd weight_;
  Eigen::MatrixXd logDencity_;
};

class KalmanFilter : public StateEstimateFilter
{
public:  // constructors & destoructors
  KalmanFilter();
  KalmanFilter(ros::NodeHandle& nh);
  KalmanFilter(ros::NodeHandle& nh,
                 const Eigen::MatrixXd system_a, const Eigen::MatrixXd system_b, const Eigen::MatrixXd system_c,
                 const int num_particle, const Eigen::MatrixXd state, const Eigen::MatrixXd weight);

public:                                                // methods
  void predict(const Eigen::MatrixXd vec_input_curr);      // 予測ステップ(事前推定)
  void filter(const Eigen::MatrixXd vec_observation_curr);                                       // フィルタリングステップ(事後推定)

private:
  void initParicles(const int num, const Eigen::MatrixXd state, const Eigen::MatrixXd weight);

private:
  Eigen::MatrixXd system_a_;
  Eigen::MatrixXd system_b_;
  Eigen::MatrixXd system_c_;
  Eigen::MatrixXd system_noise_var_;
  Eigen::MatrixXd observe_noise_var_;

  Eigen::MatrixXd prior_var_;
  Eigen::MatrixXd post_var_curr_;
  Eigen::MatrixXd post_var_prev_;

  std::vector<Particle2> particles_;
  std::default_random_engine gen_system_;
  std::normal_distribution<double> dist_system_;
  probability_distribution::ProbabilityDistribution* prob_dis_;
};
}

#endif  // PARTICLE_FILTER_H
