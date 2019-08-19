#include <image_histogram_picker/image_histogram_picker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

using image_histogram_picker::ImageHistogramPicker;

ImageHistogramPicker::ImageHistogramPicker(ros::NodeHandle& nh)
  :nh_(nh), image_sub_cnt_curr_(0)
{
  image_transport::ImageTransport it(nh);
  img_sub_ = it.subscribe("/kinect2/sd/image_depth_rect", 1,
                &ImageHistogramPicker::imageCb, this);
}

void ImageHistogramPicker::imageCb(const sensor_msgs::Image::ConstPtr& msg)
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

  cv::Mat cv_img_ori_u(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_8U);
  cv_img_ori_f_ptr->image.convertTo(cv_img_ori_u, CV_8UC1, 255.0/max_f);

  cv::Mat cv_img_hist_equ_u(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);
  cv::equalizeHist(cv_img_ori_u, cv_img_hist_equ_u);

  const char *file_name = "/home/nishidalab/temp/kinect_observe.csv";
  std::ofstream ofs;
  ofs.open(file_name, std::ios::app);

  //for(int y = 195; y < 205; y++)
  //{
  //    auto a0 = cv_img_ori_f.at<float>(y, x);

  //}

  uint a01 = cv_img_hist_equ_u.at<uchar>(200, 258);
  uint a02 = cv_img_hist_equ_u.at<uchar>(201, 258);
  uint a03 = cv_img_hist_equ_u.at<uchar>(202, 258);
  uint a04 = cv_img_hist_equ_u.at<uchar>(203, 258);
  uint a05 = cv_img_hist_equ_u.at<uchar>(204, 258);
  uint a06 = cv_img_hist_equ_u.at<uchar>(205, 258);
  uint a07 = cv_img_hist_equ_u.at<uchar>(206, 258);
  uint a08 = cv_img_hist_equ_u.at<uchar>(207, 258);
  uint a09 = cv_img_hist_equ_u.at<uchar>(208, 258);
  uint a10 = cv_img_hist_equ_u.at<uchar>(209, 258);
  uint a11 = cv_img_hist_equ_u.at<uchar>(210, 258);
  uint a12 = cv_img_hist_equ_u.at<uchar>(211, 258);
  uint a13 = cv_img_hist_equ_u.at<uchar>(212, 258);
  uint a14 = cv_img_hist_equ_u.at<uchar>(213, 258);
  uint a15 = cv_img_hist_equ_u.at<uchar>(214, 258);
  uint a16 = cv_img_hist_equ_u.at<uchar>(215, 258);
  uint a17 = cv_img_hist_equ_u.at<uchar>(216, 258);
  uint a18 = cv_img_hist_equ_u.at<uchar>(217, 258);
  uint a19 = cv_img_hist_equ_u.at<uchar>(218, 258);
  uint a20 = cv_img_hist_equ_u.at<uchar>(219, 258);

  // uint a01 = cv_img_ori_u.at<uchar>(115, 123);
  // uint a02 = cv_img_ori_u.at<uchar>(115, 124);
  // uint a03 = cv_img_ori_u.at<uchar>(115, 125);
  // uint a04 = cv_img_ori_u.at<uchar>(115, 126);
  // uint a05 = cv_img_ori_u.at<uchar>(115, 127);
  // uint a06 = cv_img_ori_u.at<uchar>(115, 128);
  // uint a07 = cv_img_ori_u.at<uchar>(115, 129);
  // uint a08 = cv_img_ori_u.at<uchar>(115, 130);
  // uint a09 = cv_img_ori_u.at<uchar>(115, 131);
  // uint a10 = cv_img_ori_u.at<uchar>(115, 132);

  // uint a01 = cv_img_ori_f.at<float>(115, 123);
  // uint a02 = cv_img_ori_f.at<float>(115, 124);
  // uint a03 = cv_img_ori_f.at<float>(115, 125);
  // uint a04 = cv_img_ori_f.at<float>(115, 126);
  // uint a05 = cv_img_ori_f.at<float>(115, 127);
  // uint a06 = cv_img_ori_f.at<float>(115, 128);
  // uint a07 = cv_img_ori_f.at<float>(115, 129);
  // uint a08 = cv_img_ori_f.at<float>(115, 130);
  // uint a09 = cv_img_ori_f.at<float>(115, 131);
  // uint a10 = cv_img_ori_f.at<float>(115, 132);

  // uint a01 = cv_img_ori_u.at<uchar>(195, 314);
  // uint a02 = cv_img_ori_u.at<uchar>(196, 314);
  // uint a03 = cv_img_ori_u.at<uchar>(197, 314);
  // uint a04 = cv_img_ori_u.at<uchar>(198, 314);
  // uint a05 = cv_img_ori_u.at<uchar>(199, 314);
  // uint a06 = cv_img_ori_u.at<uchar>(200, 314);
  // uint a07 = cv_img_ori_u.at<uchar>(201, 314);
  // uint a08 = cv_img_ori_u.at<uchar>(202, 314);
  // uint a09 = cv_img_ori_u.at<uchar>(203, 314);
  // uint a10 = cv_img_ori_u.at<uchar>(204, 314);

  //auto a01 = cv_img_ori_f.at<float>(195, 314);
  //auto a02 = cv_img_ori_f.at<float>(196, 314);
  //auto a03 = cv_img_ori_f.at<float>(197, 314);
  //auto a04 = cv_img_ori_f.at<float>(198, 314);
  //auto a05 = cv_img_ori_f.at<float>(199, 314);
  //auto a06 = cv_img_ori_f.at<float>(200, 314);
  //auto a07 = cv_img_ori_f.at<float>(201, 314);
  //auto a08 = cv_img_ori_f.at<float>(202, 314);
  //auto a09 = cv_img_ori_f.at<float>(203, 314);
  //auto a10 = cv_img_ori_f.at<float>(204, 314);
  ofs << count_result_++
      << "," << a01 << "," << a02 << "," << a03
      << "," << a04 << "," << a05 << "," << a06
      << "," << a07 << "," << a08 << "," << a09
      << "," << a10 << "," << a11 << "," << a12
      << "," << a13 << "," << a14 << "," << a15
      << "," << a16 << "," << a17 << "," << a18
      << "," << a19 << "," << a20
      << std::endl;

  ofs.close();


  cv::MatConstIterator_<float> it = cv_img_ori_f.begin<float>(), it_end = cv_img_ori_f.end<float>();
  for(; it!=it_end; it++)
  {
    auto x = (*it);
    int wait = 0;
  }
  float max_val = *std::max_element(it, it_end);
  cv_img_ori_f_ptr->image.convertTo(cv_img_ori_u, CV_8UC1, 255.0/max_f);

  //cv::Mat cv_img_only_noise(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_32FC1);
  cv::Mat cv_img_noise_mixed_u = cv_img_ori_u.clone();

  //cv::randn(cv_img_only_noise, 0., 50.);
  //cv::Mat cv_img_only_noise_u(cv_img_only_noise.rows, cv_img_only_noise.cols, CV_8UC1);
  //cv_img_only_noise.convertTo(cv_img_only_noise_u, CV_8UC1);

  //cv_img_noise_mixed_u = cv_img_ori_u + cv_img_only_noise_u;

  cv::Mat cv_img_edge_u(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);

  //cv::Mat cv_img_ori_blur_u(cv_img_ori_f_ptr->image.rows, cv_img_ori_f_ptr->image.cols, CV_8U);
  // cv::blur(cv_img_ori_u, cv_img_ori_blur_u, cv::Size(3, 3));
  //cv::Sobel(cv_img_ori_u, cv_img_edge_u, CV_8UC1, 1, 1, 3);
  cv::Mat cv_img_edge_x_u(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);
  cv::Mat cv_img_edge_x_f(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_32FC1);
  cv::Mat cv_img_edge_y_u(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);
  cv::Mat cv_img_edge_y_f(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_32FC1);

  // x derivative
  cv::Mat filterx_l = (cv::Mat_<double>(3,3) <<  0.0,  0.0,  0.0,
                                                -1.0,  1.0,  0.0,
                                                 0.0,  0.0,  0.0);
  cv::Mat filterx_r = (cv::Mat_<double>(3,3) <<  0.0,  0.0,  0.0,
                                                 0.0, -1.0,  1.0,
                                                 0.0,  0.0,  0.0);
  cv::Mat filterx;
  //cv::Sobel(cv_img_ori_u, cv_img_edge_x_f, CV_32FC1, 1, 0);
  filter2D(cv_img_ori_u, cv_img_edge_x_f, CV_32FC1, filterx_l);
  cv::convertScaleAbs(cv_img_edge_x_f, cv_img_edge_x_f);
  cv_img_edge_x_f.convertTo(cv_img_edge_x_u, CV_8UC1);

  // y derivative
  cv::Mat filtery_u = (cv::Mat_<double>(3,3) <<  0.0, -1.0,  0.0,
                                                 0.0,  1.0,  0.0,
                                                 0.0,  0.0,  0.0);
  cv::Mat filtery_d = (cv::Mat_<double>(3,3) <<  0.0,  0.0,  0.0,
                                                 0.0, -1.0,  0.0,
                                                 0.0,  1.0,  0.0);
  cv::Mat filtery;
  //cv::Sobel(cv_img_ori_u, cv_img_edge_y_f, CV_32FC1, 0, 1);
  filter2D(cv_img_ori_u, cv_img_edge_y_f, CV_32FC1, filtery_u);
  cv::convertScaleAbs(cv_img_edge_y_f, cv_img_edge_y_f);
  cv_img_edge_y_f.convertTo(cv_img_edge_y_u, CV_8UC1);
  cv_img_edge_u = cv::max(cv_img_edge_x_u, cv_img_edge_y_u);//(cv_img_edge_x_u + cv_img_edge_y_u);
  //cv::Laplacian(cv_img_ori_blur_u, cv_img_edge_u, CV_8UC1);
  //cv::Laplacian(cv_img_ori_u, cv_img_edge_u, CV_8UC1);
  //cv::Canny(cv_img_ori_u, cv_img_edge_u, 50, 200);

  cv::Mat cv_img_bin_u = cv::Mat::zeros(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);

  cv::threshold(cv_img_edge_u, cv_img_bin_u, 1, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  cv::Mat cv_img_edge_expand_u = cv::Mat::zeros(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);

  // pf init
  if(init_flg_)
  {

    init_flg_ = false;
  }

  // auto a = cv_img_ori_f_ptr->image.at<unsigned char>(250, 200);
  // auto b = cv_img_ori_u.at<unsigned char>(250, 200);
  // //auto c = cv_img_only_noise.at<unsigned char>(250, 200);
  // //auto d = cv_img_only_noise_u.at<unsigned char>(250, 200);
  // auto e = cv_img_noise_mixed_u.at<unsigned char>(250, 200);
  // auto f = cv_img_edge_u.at<unsigned char>(250, 200);

  cv::imshow("cv_img_ori_u", cv_img_ori_u);
  cv::imshow("cv_img_ori_f", cv_img_ori_f);
  cv::imshow("cv_img_hist_equ_u", cv_img_hist_equ_u);
  // cv::imwrite( "/home/nishidalab/Pictures/depth_pf/cv_img_ori_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_ori_u );
  // //cv::imshow("cv_img_ori_blur_u", cv_img_ori_blur_u);
  // //cv::imshow("cv_img_only_noise_u", cv_img_only_noise_u);
  // cv::imshow("cv_img_noise_mixed_u", cv_img_noise_mixed_u);
  // cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_mixed_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_noise_mixed_u);
  // //cv::imshow("cv_img_edge_x_u", cv_img_edge_x_u);
  // //cv::imshow("cv_img_edge_y_u", cv_img_edge_y_u);
  // //cv::imshow("cv_img_edge_u", cv_img_edge_u);
  // cv::imshow("cv_img_edge_expand_u", cv_img_edge_expand_u);
  // cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_edge_expand_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_edge_expand_u);
  // //cv::imshow("cv_img_bin_u", cv_img_bin_u);
  // cv::imshow("cv_img_filtered_u", cv_img_filtered_u);
  // cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_filtered_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_filtered_u);
  // cv::imshow("cv_img_filtered_f", cv_img_filtered_f);
  // cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_filtered_f_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_filtered_f);

  // // publisher
  // sensor_msgs::ImagePtr msg_img_noise_mixed;

  // msg_img_noise_mixed = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_img_noise_mixed_u).toImageMsg();
  // img_pub_.publish(msg_img_noise_mixed);

  image_sub_cnt_curr_++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_ros");
  ros::NodeHandle nh;
  ImageHistogramPicker ihp(nh);

  ros::Rate looprate (5);   // read image at 5Hz
  while (ros::ok())
  {
    if (cv::waitKey(1) == 'q')
      break;

    if (ihp.image_sub_cnt_curr_ > 100)
      break;

    ros::spinOnce();
    looprate.sleep();
  }
  return 0;
}
