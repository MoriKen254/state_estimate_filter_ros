#ifndef STATE_ESTIMATE_FILTER_H
#define STATE_ESTIMATE_FILTER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

namespace state_estimate_filter_ros
{
class StateEstimateFilter
{
public:
  StateEstimateFilter(ros::NodeHandle& nh);
  virtual ~StateEstimateFilter();

  void broadcast(void);
  void estimate(const Eigen::MatrixXd vec_input_curr);
  virtual void predict(const Eigen::MatrixXd vec_input_curr) = 0;
  virtual void filter(void) = 0;

private:
  void fixedPhoxiPoseCallback(const geometry_msgs::PoseStamped fixed_phoxi_pose);

private:
  ros::NodeHandle nh_;
  ros::Subscriber fixed_phoxi_sub_;
  tf::Transform fixed_phoxi_tf_;
  tf::TransformBroadcaster br_;
  bool init_flg_;
};
}

#endif
