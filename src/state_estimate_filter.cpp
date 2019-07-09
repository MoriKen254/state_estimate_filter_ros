#include <state_estimate_filter_ros/state_estimate_filter.h>

using state_estimate_filter_ros::StateEstimateFilter;

// Class methods definitions
StateEstimateFilter::StateEstimateFilter()
{
}

StateEstimateFilter::StateEstimateFilter(ros::NodeHandle& nh) : nh_(nh)
{
}

void StateEstimateFilter::estimate(const Eigen::MatrixXd vec_input_curr)
{
  this->predict(vec_input_curr);  // 一段先予測
  this->filter();                 // フィルタリング
}
