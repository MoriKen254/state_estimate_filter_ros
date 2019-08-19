#ifndef ONLY_ONE_PIX_PF_ANALIZER_H
#define ONLY_ONE_PIX_PF_ANALIZER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <state_estimate_filter_ros/particle_filter.h>

namespace only_one_pix_pf_analizer
{
class OnlyOnePixPfAnalizer
{
public:
  OnlyOnePixPfAnalizer(ros::NodeHandle& nh);
  void analyzeOnePix(void);

private:
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);
  std::vector<std::string> split(std::string& input, char delimiter);

public:
  int image_sub_cnt_curr_;

private:
  ros::NodeHandle nh_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
  std::vector<state_estimate_filter_ros::StateEstimateFilter*> particle_filters;
  bool init_flg_;
  int count_result_;
};
}  // namespace only_one_pix_pf_analizer

#endif  // ONLY_ONE_PIX_PF_ANALIZER_H
