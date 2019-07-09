#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "state_estimate_filter.h"

namespace state_estimate_filter_ros
{
class ParticleFilter : StateEstimateFilter
{
public:  // constructors & destoructors
  ParticleFilter();
  ParticleFilter(ros::NodeHandle& nh);
  ParticleFilter(ros::NodeHandle& nh, const Eigen::MatrixXd system_a, const Eigen::MatrixXd system_b);

public:                                                // methods
  void predict(const Eigen::MatrixXd vec_input_curr);  // 予測ステップ(事前推定)
  void filter();                                       // フィルタリングステップ(事後推定)

private:
  Eigen::MatrixXd system_a_;
  Eigen::MatrixXd system_b_;
  double predicted_val_;
};
}

#endif  // PARTICLE_FILTER_H
