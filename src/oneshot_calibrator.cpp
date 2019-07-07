#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string>
#include <vector>
#include <cmath>

geometry_msgs::Pose initPose(float x, float y, float z, float rx, float ry, float rz, float rw)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = rx;
  pose.orientation.y = ry;
  pose.orientation.z = rz;
  pose.orientation.w = rw;

  return pose;
}

// Calculate center points of multiple points
geometry_msgs::PoseStamped getCenterGeometry(std::vector<geometry_msgs::Pose> point)
{
  geometry_msgs::PoseStamped point_center;

  for (std::size_t i = 0; i < point.size(); i++)
  {
    point_center.pose.position.x = i * point_center.pose.position.x / (i + 1) + point[i].position.x / (i + 1);
    point_center.pose.position.y = i * point_center.pose.position.y / (i + 1) + point[i].position.y / (i + 1);
    point_center.pose.position.z = i * point_center.pose.position.z / (i + 1) + point[i].position.z / (i + 1);
    point_center.pose.orientation.x = i * point_center.pose.orientation.x / (i + 1) + point[i].orientation.x / (i + 1);
    point_center.pose.orientation.y = i * point_center.pose.orientation.y / (i + 1) + point[i].orientation.y / (i + 1);
    point_center.pose.orientation.z = i * point_center.pose.orientation.z / (i + 1) + point[i].orientation.z / (i + 1);
    point_center.pose.orientation.w = i * point_center.pose.orientation.w / (i + 1) + point[i].orientation.w / (i + 1);
  }

  return point_center;
}

void arAndPhoxiPoseCallback(ros::Publisher& fixed_phoxi_pub, const geometry_msgs::PoseArray::ConstPtr& ar_pose,
                            const geometry_msgs::PoseArray::ConstPtr& phoxi_pose)
{
  // true AR marker TF
  geometry_msgs::PoseArray true_ar_pose;
  true_ar_pose.poses.push_back(initPose(0.5675, -0.2975, 0.0, 0.0, 0.0, 0.0, 1.0));
  true_ar_pose.poses.push_back(initPose(0.0675, -0.2975, 0.0, 0.0, 0.0, 0.0, 1.0));
  true_ar_pose.poses.push_back(initPose(0.0675, 0.2975, 0.0, 0.0, 0.0, 0.0, 1.0));
  true_ar_pose.poses.push_back(initPose(0.5675, 0.2975, 0.0, 0.0, 0.0, 0.0, 1.0));

  // true AR marker TF
  geometry_msgs::PoseStamped true_ar_center_pose = getCenterGeometry(true_ar_pose.poses);

  // AR marker TF
  geometry_msgs::PoseStamped ar_center_pose = getCenterGeometry(ar_pose->poses);

  // phoxi TF
  geometry_msgs::PoseStamped phoxi_center_pose = getCenterGeometry(phoxi_pose->poses);

  // modify transform
  float bias_x = true_ar_center_pose.pose.position.x - ar_center_pose.pose.position.x;
  float bias_y = true_ar_center_pose.pose.position.y - ar_center_pose.pose.position.y;
  float bias_z = true_ar_center_pose.pose.position.z - ar_center_pose.pose.position.z;
  float bias_rx = true_ar_center_pose.pose.orientation.x - ar_center_pose.pose.orientation.x;
  float bias_ry = true_ar_center_pose.pose.orientation.y - ar_center_pose.pose.orientation.y;
  float bias_rz = true_ar_center_pose.pose.orientation.z - ar_center_pose.pose.orientation.z;
  float bias_rw = true_ar_center_pose.pose.orientation.w - ar_center_pose.pose.orientation.w;

  // fixed phoxi TF
  geometry_msgs::PoseStamped fixed_phoxi;
  fixed_phoxi.header.stamp = phoxi_pose->header.stamp;
  fixed_phoxi.pose.position.x = phoxi_center_pose.pose.position.x + bias_x;
  fixed_phoxi.pose.position.y = phoxi_center_pose.pose.position.y + bias_y;
  fixed_phoxi.pose.position.z = phoxi_center_pose.pose.position.z + bias_z;
  fixed_phoxi.pose.orientation.x = phoxi_center_pose.pose.orientation.x - bias_rx;
  fixed_phoxi.pose.orientation.y = phoxi_center_pose.pose.orientation.y - bias_ry;
  fixed_phoxi.pose.orientation.z = phoxi_center_pose.pose.orientation.z - bias_rz;
  fixed_phoxi.pose.orientation.w = phoxi_center_pose.pose.orientation.w - bias_rw;

  // publish
  fixed_phoxi_pub.publish(fixed_phoxi);
  ROS_INFO("successed !");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "oneshot_calibration");
  ros::NodeHandle nh;

  ros::Publisher fixed_phoxi_pub = nh.advertise<geometry_msgs::PoseStamped>("/fixed_phoxi_pose", 1);

  message_filters::Subscriber<geometry_msgs::PoseArray> ar_sub(nh, "ar_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> phoxi_sub(nh, "phoxi_pose", 1);
  message_filters::TimeSynchronizer<geometry_msgs::PoseArray, geometry_msgs::PoseArray> sync(ar_sub, phoxi_sub, 10);
  sync.registerCallback(boost::bind(arAndPhoxiPoseCallback, fixed_phoxi_pub, _1, _2));

  ros::spin();

  return 0;
}
