#include <state_estimate_filter_ros/state_estimate_filter.h>

using state_estimate_filter_ros::StateEstimateFilter;

// Class methods definitions
StateEstimateFilter::StateEstimateFilter(ros::NodeHandle& nh) : nh_(nh), init_flg_(false)
{
  fixed_phoxi_sub_ = nh.subscribe("fixed_phoxi_pose", 1, &StateEstimateFilter::fixedPhoxiPoseCallback, this);
}

void StateEstimateFilter::broadcast(void)
{
  // Never broadcast tf unless at least once fixed_phoxi_pose is subscribed
  if (!init_flg_)
    return;

  br_.sendTransform(tf::StampedTransform(fixed_phoxi_tf_, ros::Time::now(), "base_link", "fixed_phoxi_tf"));
}


void StateEstimateFilter::estimate(const Eigen::MatrixXd vec_input_curr)
{
  this->predict(vec_input_curr);  // 一段先予測
  this->filter(); // フィルタリング
}

void StateEstimateFilter::fixedPhoxiPoseCallback(const geometry_msgs::PoseStamped fixed_phoxi_pose)
{
  init_flg_ = true;

  // Position
  tf::Vector3 tf_vec =
      tf::Vector3(fixed_phoxi_pose.pose.position.x, fixed_phoxi_pose.pose.position.y, fixed_phoxi_pose.pose.position.z);
  fixed_phoxi_tf_.setOrigin(tf_vec);

  // Posture
  tf::Quaternion fixed_phoxi_quat(fixed_phoxi_pose.pose.orientation.x, fixed_phoxi_pose.pose.orientation.y,
                                  fixed_phoxi_pose.pose.orientation.z, fixed_phoxi_pose.pose.orientation.w);
  fixed_phoxi_tf_.setRotation(fixed_phoxi_quat);
}
