#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>

geometry_msgs::Pose getPoseFromTF(tf::StampedTransform input_tf)
{
  geometry_msgs::Pose pose;
  pose.position.x = input_tf.getOrigin().x();
  pose.position.y = input_tf.getOrigin().y();
  pose.position.z = input_tf.getOrigin().z();
  pose.orientation.x = input_tf.getRotation().x();
  pose.orientation.y = input_tf.getRotation().y();
  pose.orientation.z = input_tf.getRotation().z();
  pose.orientation.w = input_tf.getRotation().w();

  return pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_pose_publisher");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener;

  ros::Publisher ar_pose_pub = nh.advertise<geometry_msgs::PoseArray>("ar_pose", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // listen ar_marker_tf
    tf::StampedTransform ar0_tf, ar1_tf, ar2_tf, ar3_tf;
    while (ros::ok())
    {
      try
      {
        tf_listener.waitForTransform("/base_link", "/ar_marker_0", ros::Time(0), ros::Duration(8.0));
        tf_listener.lookupTransform("/base_link", "/ar_marker_0", ros::Time(0), ar0_tf);
        tf_listener.waitForTransform("/base_link", "/ar_marker_2", ros::Time(0), ros::Duration(8.0));
        tf_listener.lookupTransform("/base_link", "/ar_marker_2", ros::Time(0), ar1_tf);
        tf_listener.waitForTransform("/base_link", "/ar_marker_3", ros::Time(0), ros::Duration(8.0));
        tf_listener.lookupTransform("/base_link", "/ar_marker_3", ros::Time(0), ar2_tf);
        tf_listener.waitForTransform("/base_link", "/ar_marker_5", ros::Time(0), ros::Duration(8.0));
        tf_listener.lookupTransform("/base_link", "/ar_marker_5", ros::Time(0), ar3_tf);

        ROS_INFO("successed !");
        break;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    // convert to PoseArray from StampedTransform
    geometry_msgs::PoseArray ar_pose;
    ar_pose.header.stamp = ros::Time(0);
    ar_pose.poses.push_back(getPoseFromTF(ar0_tf));
    ar_pose.poses.push_back(getPoseFromTF(ar1_tf));
    ar_pose.poses.push_back(getPoseFromTF(ar2_tf));
    ar_pose.poses.push_back(getPoseFromTF(ar3_tf));

    // publish
    ar_pose_pub.publish(ar_pose);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
