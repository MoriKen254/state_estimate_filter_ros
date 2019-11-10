#ifndef IMAGE_ZEROPIX_FILLER_H
#define IMAGE_ZEROPIX_FILLER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <state_estimate_filter_ros/particle_filter.h>
#include <state_estimate_filter_ros/kalman_filter.h>

namespace image_zeropix_filler
{
class ImageZeroPixFiller
{
public:
  ImageZeroPixFiller(ros::NodeHandle& nh);

private:
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);

public:
  int image_sub_cnt_curr_;

private:
  ros::NodeHandle nh_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
  std::vector<state_estimate_filter_ros::StateEstimateFilter*> particle_filters_;
  std::vector<state_estimate_filter_ros::StateEstimateFilter*> kalman_filters_;
  bool init_flg_;
  int count_result_;
};
}  // namespace image_zeropix_filler

#endif  // IMAGE_ZEROPIX_FILLER_H
