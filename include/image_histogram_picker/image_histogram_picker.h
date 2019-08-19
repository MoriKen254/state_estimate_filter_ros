#ifndef IMAGE_HISTOGRAM_PICKER_H
#define IMAGE_HISTOGRAM_PICKER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace image_histogram_picker
{
class ImageHistogramPicker
{
public:
  ImageHistogramPicker(ros::NodeHandle& nh);

private:
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);

public:
  int image_sub_cnt_curr_;

private:
  ros::NodeHandle nh_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
  bool init_flg_;
  int count_result_;
};
}  // namespace image_noise_mixer

#endif  // IMAGE_HISTOGRAM_PICKER_H
