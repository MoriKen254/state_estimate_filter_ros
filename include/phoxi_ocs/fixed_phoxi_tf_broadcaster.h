#ifndef FIXED_PHOXI_TF_BROADCASTER_H
#define FIXED_PHOXI_TF_BROADCASTER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

class FixedPhoxiTFBroadcaster
{
public:
  FixedPhoxiTFBroadcaster(ros::NodeHandle& nh);

  void broadcast(void);

private:
  void fixedPhoxiPoseCallback(const geometry_msgs::PoseStamped fixed_phoxi_pose);

private:
  ros::NodeHandle nh_;
  ros::Subscriber fixed_phoxi_sub_;
  tf::Transform fixed_phoxi_tf_;
  tf::TransformBroadcaster br_;
  bool init_flg_;
};

#endif
