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
  virtual void predict(const Eigen::MatrixXd vec_input_curr) = 0;
  virtual void filter(void) = 0;

private:
  ros::NodeHandle nh_;
};
}

#endif
