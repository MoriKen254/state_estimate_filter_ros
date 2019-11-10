#include <image_zeropix_filler/image_zeropix_filler.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <chrono>

#include <fstream>

using Eigen::MatrixXd;
using image_zeropix_filler::ImageZeroPixFiller;
using state_estimate_filter_ros::StateEstimateFilter;
using state_estimate_filter_ros::ParticleFilter;
using state_estimate_filter_ros::KalmanFilter;

ImageZeroPixFiller::ImageZeroPixFiller(ros::NodeHandle& nh)
  : nh_(nh), init_flg_(true), image_sub_cnt_curr_(0), count_result_(0)
{
  image_transport::ImageTransport it(nh);
  // img_sub_ = it.subscribe("/kinect2/hd/image_depth", 1, // for real
  img_sub_ = it.subscribe("/kinect2/sd/image_depth", 1, // for real
                &ImageZeroPixFiller::imageCb, this);
  img_pub_ = it.advertise("/image_noise_mixer/image_depth_noise", 1);

}

void ImageZeroPixFiller::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_img_ori_f_ptr;
  try {
    cv_img_ori_f_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  cv::Mat cv_img_ori_f(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, cv_img_ori_f_ptr->image.type());
  cv_img_ori_f = cv_img_ori_f_ptr->image;

  double max_f = 0;
  int width = cv_img_ori_f.cols;
  int height = cv_img_ori_f.rows;
  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      auto a0 = cv_img_ori_f.at<float>(y, x);
      int wait = 0;
      if (!std::isnan(a0))
      {
        int wait2 = 0;
      }

      if (std::isnan(a0))
        break;

      if(a0 > max_f)
        max_f = a0;
    }
  }

  cv::Mat cv_img_zerofilled_f(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, cv_img_ori_f_ptr->image.type());

  cv::Mat cv_img_ori_u(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_8U);
  cv_img_ori_f_ptr->image.convertTo(cv_img_ori_u, CV_8UC1, 255.0/max_f);


  cv::Mat cv_img_hist_equ_u(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_8U);
  cv::equalizeHist(cv_img_ori_u, cv_img_hist_equ_u);

  cv::Mat cv_img_zerofilled_u = cv_img_hist_equ_u.clone();

  cv::medianBlur(cv_img_hist_equ_u, cv_img_zerofilled_u, 5);

  //for(int y = 0; y < cv_img_ori_u.rows; y++)
  //{
  //  for(int x = 0; x < cv_img_ori_u.cols; x++)
  //  {
  //    auto a0 = cv_img_ori_u.at<float>(y, x);
  //    int wait = 0;
  //    //if (!std::isnan(a0))
  //    //{
  //    //  int wait2 = 0;
  //    //}

  //    if (std::isnan(a0) || a0 < 10)
  //    {
  //      int y_base = y;
  //      int x_base = x;
  //      cv::Rect roi(x_base, y_base, 10, 10);
  //      cv::Mat ori_roi = cv_img_ori_u(roi);
  //      cv::Mat filled_roi = cv_img_zerofilled_u(roi);

  //      double mMin, mMax;
  //      cv::Point minP, maxP;
  //      cv::minMaxLoc(ori_roi, &mMin, &mMax, &minP, &maxP);
  //      std::cout << ori_roi << std::endl;
  //      std::cout << "min: " << mMin << ", point " << minP << std::endl;
  //      std::cout << "max: " << mMax << ", point " << maxP << std::endl;


  //    }

  //    if(a0 > max_f)
  //      max_f = a0;
  //  }
  // }


  cv::imwrite("/home/nishidalab/Pictures/depth_pf/191110/cv_img_ori_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_ori_u);
  cv::imwrite("/home/nishidalab/Pictures/depth_pf/191110/cv_img_hist_equ_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_hist_equ_u);
  cv::imwrite("/home/nishidalab/Pictures/depth_pf/191110/cv_img_zerofilled_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_zerofilled_u);

  // publisher
  sensor_msgs::ImagePtr msg_img_zerofilled;

  msg_img_zerofilled = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_img_zerofilled_u).toImageMsg();
  img_pub_.publish(msg_img_zerofilled);

  image_sub_cnt_curr_++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_ros");
  ros::NodeHandle nh;
  ImageZeroPixFiller inm(nh);

  ros::Rate looprate (5);   // read image at 5Hz
  while (ros::ok())
  {
    if (cv::waitKey(1) == 'q')
      break;

    if (inm.image_sub_cnt_curr_ > 100)
      break;

    ros::spinOnce();
    looprate.sleep();
  }
  return 0;
}
