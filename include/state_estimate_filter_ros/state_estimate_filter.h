#ifndef STATE_ESTIMATE_FILTER_H
#define STATE_ESTIMATE_FILTER_H

#include <ros/ros.h>
#include <Eigen/Dense>

namespace state_estimate_filter_ros
{
class StateEstimateFilter
{
public:
  StateEstimateFilter();
  StateEstimateFilter(ros::NodeHandle& nh);
  virtual ~StateEstimateFilter();

  void estimate(const Eigen::MatrixXd vec_input_curr);
  void setInitVal(const Eigen::MatrixXd vec_init_val);
  virtual void predict(const Eigen::MatrixXd vec_input_curr) = 0;
  virtual void filter(void) = 0;

 public:
  ros::NodeHandle nh_;
  Eigen::MatrixXd vec_predict_curr_;
  Eigen::MatrixXd vec_predict_prev_;
  Eigen::MatrixXd vec_estimate_curr_;
  Eigen::MatrixXd vec_estimate_prev_;
};
}

#endif
