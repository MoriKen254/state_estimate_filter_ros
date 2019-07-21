#ifndef IMAGE_NOISE_MIXER_H
#define IMAGE_NOISE_MIXER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <state_estimate_filter_ros/particle_filter.h>

namespace image_noise_mixer
{
class ImageNoiseMixer
{
public:
  ImageNoiseMixer(ros::NodeHandle& nh);

private:
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);

public:
  int image_sub_cnt_curr_;

private:
  ros::NodeHandle nh_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
  std::vector<state_estimate_filter_ros::StateEstimateFilter*> particle_filters;
  bool init_flg_;
};
}  // namespace image_noise_mixer

#endif  // IMAGE_NOISE_MIXER_H
