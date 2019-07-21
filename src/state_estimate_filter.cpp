#include <state_estimate_filter_ros/state_estimate_filter.h>

using state_estimate_filter_ros::StateEstimateFilter;
using Eigen::MatrixXd;

// Class methods definitions
StateEstimateFilter::StateEstimateFilter()
{
}

StateEstimateFilter::~StateEstimateFilter()
{
}

StateEstimateFilter::StateEstimateFilter(ros::NodeHandle& nh) : nh_(nh)
{
  vec_estimate_prev_ = MatrixXd::Zero(1, 1);
  vec_estimate_curr_ = MatrixXd::Zero(1, 1);
  vec_predict_prev_ = MatrixXd::Zero(1, 1);
  vec_predict_curr_ = MatrixXd::Zero(1, 1);
}

void StateEstimateFilter::setInitVal(const MatrixXd vec_init_val)
{
  vec_estimate_curr_ = vec_init_val;
}

void StateEstimateFilter::estimate(const Eigen::MatrixXd vec_input_curr)
{
  this->predict(vec_input_curr);  // 一段先予測
  this->filter();                 // フィルタリング
}
