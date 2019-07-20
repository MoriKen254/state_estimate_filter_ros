#include <image_noise_mixer/image_noise_mixer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using image_noise_mixer::ImageNoiseMixer;

ImageNoiseMixer::ImageNoiseMixer(ros::NodeHandle& nh)
  : nh_(nh)
{
  image_transport::ImageTransport it(nh);
  img_sub_ = it.subscribe("/kinect2/hd/image_depth", 1,
                &ImageNoiseMixer::imageCb, this);
  img_pub_ = it.advertise("/image_noise_mixer/image_depth_noise", 1);
}

void ImageNoiseMixer::imageCb(const sensor_msgs::Image::ConstPtr& msg)
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

  float max_f = 0;
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

  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      if (std::isnan(cv_img_ori_f.at<float>(y, x)))
      {
        cv_img_ori_f.at<float>(y, x) = max_f;
        auto a0 = cv_img_ori_f.at<float>(y, x);
        int wait = 0;
      }
    }
  }

  // adding noise to (simulated) depth image
  cv::Mat cv_img_ori_u(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_8U);

  cv::MatConstIterator_<float> it = cv_img_ori_f.begin<float>(), it_end = cv_img_ori_f.end<float>();
  for(; it!=it_end; it++)
  {
    auto x = (*it);
    int wait = 0;
  }
  float max_val = *std::max_element(it, it_end);
  cv_img_ori_f_ptr->image.convertTo(cv_img_ori_u, CV_8UC1, 255.0/max_f);

  cv::Mat cv_img_only_noise(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_32FC1);
  cv::Mat cv_img_noise_mixed_u(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_8UC1);

  cv::randn(cv_img_only_noise, 0., 50.);
  cv::Mat cv_img_only_noise_u(cv_img_only_noise.rows, cv_img_only_noise.cols, CV_8UC1);
  cv_img_only_noise.convertTo(cv_img_only_noise_u, CV_8UC1);

  cv_img_noise_mixed_u = cv_img_ori_u + cv_img_only_noise_u;

  cv::Mat cv_img_edge_u(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);

  cv::Sobel(cv_img_noise_mixed_u, cv_img_edge_u, CV_8UC1, 1, 1, 3);

  auto a = cv_img_ori_f_ptr->image.at<unsigned char>(250, 200);
  auto b = cv_img_ori_u.at<unsigned char>(250, 200);
  auto c = cv_img_only_noise.at<unsigned char>(250, 200);
  auto d = cv_img_only_noise_u.at<unsigned char>(250, 200);
  auto e = cv_img_noise_mixed_u.at<unsigned char>(250, 200);
  auto f = cv_img_edge_u.at<unsigned char>(250, 200);

  cv::imshow("cv_img_ori_u", cv_img_ori_u);
  cv::imshow("cv_img_only_noise_u", cv_img_only_noise_u);
  cv::imshow("cv_img_noise_mixed_u", cv_img_noise_mixed_u);
  cv::imshow("cv_img_edge_u", cv_img_edge_u);

  // publisher
  sensor_msgs::ImagePtr msg_img_noise_mixed;

  msg_img_noise_mixed = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_img_noise_mixed_u).toImageMsg();
  img_pub_.publish(msg_img_noise_mixed);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_ros");
    ros::NodeHandle nh;
    ImageNoiseMixer inm(nh);

    ros::Rate looprate (5);   // read image at 5Hz
    while (ros::ok())
    {
        if (cv::waitKey(1) == 'q')
            break;

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}