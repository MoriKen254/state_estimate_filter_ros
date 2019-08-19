#include <only_one_pix_pf_analizer/only_one_pix_pf_analizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <chrono>

#include <fstream>
#include <string>

using Eigen::MatrixXd;
using only_one_pix_pf_analizer::OnlyOnePixPfAnalizer;
using state_estimate_filter_ros::StateEstimateFilter;
using state_estimate_filter_ros::ParticleFilter;


OnlyOnePixPfAnalizer::OnlyOnePixPfAnalizer(ros::NodeHandle& nh)
  : nh_(nh), init_flg_(true), image_sub_cnt_curr_(0), count_result_(0)
{
  image_transport::ImageTransport it(nh);
  img_sub_ = it.subscribe("/kinect2/hd/image_depth", 1,
                &OnlyOnePixPfAnalizer::imageCb, this);
  img_pub_ = it.advertise("/image_noise_mixer/image_depth_noise", 1);

}

void OnlyOnePixPfAnalizer::imageCb(const sensor_msgs::Image::ConstPtr& msg)
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

    MatrixXd system_a(1, 1);
    MatrixXd system_b(1, 1);
    MatrixXd system_c(1, 1);
    system_a(0, 0) = 1.0;
    system_b(0, 0) = 0.0;
    system_c(0, 0) = 0.0;

    int num_particle = 50;
    MatrixXd vec_weight(1, 1);
    vec_weight(0, 0) = 1.0/static_cast<double>(num_particle);

    MatrixXd vec_init_val(1, 1);

    particle_filters.resize(height * width);
    int index = 0;
    for(int y = 0; y < height; y++)
    {
      for(int x = 0; x < width; x++)
      {
        vec_init_val(0, 0) = static_cast<float>(cv_img_ori_u.at<uchar>(y, x));
        StateEstimateFilter* particle_filter = new ParticleFilter(nh_,
                                                                  system_a, system_b, system_c,
                                                                  num_particle, vec_init_val, vec_weight);
        index = x + y * width;
        particle_filters[index] = particle_filter;
        // particle_filters.push_back(particle_filter);
      }
    }

    init_flg_ = false;
  }

  for(int y = 0; y < height; y++)
  {
    for(int x = 0; x < width; x++)
    {
      uchar edge_val = cv_img_edge_u.at<uchar>(y, x);
      if(edge_val < 50)//18)
        continue;

      // only overwrite when it's forground
      if(x < width / 2 && cv_img_ori_u.at<uchar>(y, x) > 100)
        continue;

      if(x >= width / 2 && cv_img_ori_u.at<uchar>(y, x) > 120)//70)
        continue;

      //cv::RNG rng(static_cast<uint64>(clock()));
      //double rand_gau = 10 * rng.gaussian(edge_val) * static_cast<double>(edge_val); //標準偏差を指定．
      ////cv_img_edge_expand_u.at<uchar>(y, x) = static_cast<uchar>(std::min(edge_val + rand_gau, 255));
      //cv_img_edge_expand_u.at<uchar>(y, x) = static_cast<uchar>(edge_val + rand_gau);

      // judge whether noise will be added or not in this step
      std::default_random_engine generator_judge;
      generator_judge.seed(std::chrono::system_clock::now().time_since_epoch().count());
      std::normal_distribution<double> distribution_judge(0.0, 1.0);
      double val_judge = distribution_judge(generator_judge);
      // judge < 0 (50%): no noise added, 0 <= judge < 1 (34%): no noise added
      if(val_judge < 1) // noise adding condition (16%)
        continue;

      //int noise = static_cast<int>(std::min<double>(rand_gau, 10.0));
      std::default_random_engine generator;
      generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
      std::normal_distribution<double> distribution(edge_val, 5.0);
      double noise = distribution(generator);
      uchar candidate;
      uint update_idx_x, update_idx_y;

      if(x < width/2)
        update_idx_x = x;
      else
        update_idx_x = x - 1;

      if(y < 221)
        update_idx_y = y;
      else
        update_idx_y = y - 1;


      candidate = static_cast<uchar>(std::max<uchar>(0, cv_img_ori_u.at<uchar>(update_idx_y, update_idx_x) + noise));

      if(edge_val < 150)
      {
        uchar n = sqrt(edge_val);
        uchar m = n;
      }

      //if(candidate > 190)
      //  continue;


      cv_img_noise_mixed_u.at<uchar>(y, x) = candidate;
      cv_img_edge_expand_u.at<uchar>(y, x) = candidate;
    }
  }


  cv::Mat cv_img_noise_mixed_f(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_32FC1);
  cv_img_noise_mixed_u.convertTo(cv_img_noise_mixed_f, CV_32FC1);

  MatrixXd vec_input(1, 1);
  MatrixXd vec_obs_curr(1, 1);
  vec_input(0, 0) = 0.0;
  cv::Mat cv_img_filtered_f(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_32FC1);

  cv::Mat cv_img_filtered_u(cv_img_ori_u.rows, cv_img_ori_u.cols, CV_8UC1);

  const char *file_name = "/home/nishidalab/temp/result.csv";
  std::ofstream ofs;
  //ofs.open(file_name, std::ios::app);
  //ofs << "count,original,with_noise,filterd" << std::endl;
  //ofs.close();

  for(int y = 0; y < height; y++)
  {
    if(y != 37)
      continue;

    for(int x = 0; x < width; x++)
    {
      if(x != 157)
        continue;
#if 1
      uchar a = cv_img_noise_mixed_u.at<uchar>(y, x);
      vec_obs_curr(0, 0) = cv_img_noise_mixed_f.at<float>(y, x);

      int index = x + y * width;
      particle_filters[index]->estimate(vec_input, vec_obs_curr);

      cv_img_filtered_f.at<float>(y, x) = particle_filters[index]->vec_estimate_curr_(0, 0);

      ofs.open(file_name, std::ios::app);

      int ori = cv_img_ori_u.at<uchar>(y, x);
      int noise = cv_img_noise_mixed_u.at<uchar>(y, x);
      cv_img_filtered_f.convertTo(cv_img_filtered_u, CV_8UC1);
      int filtered = cv_img_filtered_u.at<uchar>(y, x);
      ofs << count_result_++ << "," << ori << "," << noise << "," << filtered << std::endl;

      ofs.close();
#endif
    }
  }

  cv_img_filtered_f.convertTo(cv_img_filtered_u, CV_8UC1);

  auto a = cv_img_ori_f_ptr->image.at<unsigned char>(250, 200);
  auto b = cv_img_ori_u.at<unsigned char>(250, 200);
  //auto c = cv_img_only_noise.at<unsigned char>(250, 200);
  //auto d = cv_img_only_noise_u.at<unsigned char>(250, 200);
  auto e = cv_img_noise_mixed_u.at<unsigned char>(250, 200);
  auto f = cv_img_edge_u.at<unsigned char>(250, 200);

  cv::imshow("cv_img_ori_u", cv_img_ori_u);
  cv::imwrite( "/home/nishidalab/Pictures/depth_pf/cv_img_ori_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_ori_u );
  //cv::imshow("cv_img_ori_blur_u", cv_img_ori_blur_u);
  //cv::imshow("cv_img_only_noise_u", cv_img_only_noise_u);
  cv::imshow("cv_img_noise_mixed_u", cv_img_noise_mixed_u);
  cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_mixed_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_noise_mixed_u);
  //cv::imshow("cv_img_edge_x_u", cv_img_edge_x_u);
  //cv::imshow("cv_img_edge_y_u", cv_img_edge_y_u);
  //cv::imshow("cv_img_edge_u", cv_img_edge_u);
  cv::imshow("cv_img_edge_expand_u", cv_img_edge_expand_u);
  cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_edge_expand_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_edge_expand_u);
  //cv::imshow("cv_img_bin_u", cv_img_bin_u);
  cv::imshow("cv_img_filtered_u", cv_img_filtered_u);
  cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_filtered_u_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_filtered_u);
  cv::imshow("cv_img_filtered_f", cv_img_filtered_f);
  cv::imwrite("/home/nishidalab/Pictures/depth_pf/cv_img_filtered_f_" + std::to_string(image_sub_cnt_curr_) + ".png", cv_img_filtered_f);

  // publisher
  sensor_msgs::ImagePtr msg_img_noise_mixed;

  msg_img_noise_mixed = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_img_noise_mixed_u).toImageMsg();
  img_pub_.publish(msg_img_noise_mixed);

  image_sub_cnt_curr_++;
}

void OnlyOnePixPfAnalizer::analyzeOnePix(void)
{
  const char *file_name = "/home/nishidalab/temp/kinect_observe_u_for_real_sensor_pf.csv";
  std::ifstream ifs(file_name);

  std::string line;
  std::vector<int> values;
  while (getline(ifs, line)) {

    std::vector<std::string> strvec = split(line, ',');

    for (int i=0; i<strvec.size(); i++){
      values.push_back(stoi(strvec.at(i)));
      printf("%5d\n", stoi(strvec.at(i)));
    }
  }
  ifs.close();

  MatrixXd system_a(1, 1);
  MatrixXd system_b(1, 1);
  MatrixXd system_c(1, 1);
  system_a(0, 0) = 1.0;
  system_b(0, 0) = 0.0;
  system_c(0, 0) = 0.0;

  int num_particle = 50;
  MatrixXd vec_weight(1, 1);
  vec_weight(0, 0) = 1.0/static_cast<double>(num_particle);

  MatrixXd vec_init_val(1, 1);
  vec_init_val(0, 0) = 13;//values[0];
  StateEstimateFilter* particle_filter = new ParticleFilter(nh_,
                                                            system_a, system_b, system_c,
                                                            num_particle, vec_init_val, vec_weight);

  MatrixXd vec_input(1, 1);
  MatrixXd vec_obs_curr(1, 1);
  vec_input(0, 0) = 0.0;

  const char *file_name_out = "/home/nishidalab/temp/kinect_observe_after_pf_analize.csv";
  std::ofstream ofs;
  ofs.open(file_name_out, std::ios::app);

  for (int i : values)
  {
    vec_obs_curr(0, 0) = i;
    particle_filter->estimate(vec_input, vec_obs_curr);
    int val = particle_filter->vec_estimate_curr_(0, 0);
    ofs << val << std::endl;
  }

  ofs.close();
}

std::vector<std::string> OnlyOnePixPfAnalizer::split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_ros");
  ros::NodeHandle nh;
  OnlyOnePixPfAnalizer ooppa(nh);

  //ros::Rate looprate (5);   // read image at 5Hz
  //while (ros::ok())
  //{
  //  if (cv::waitKey(1) == 'q')
  //    break;

  //  //if (ooppa.image_sub_cnt_curr_ > 100)
  //  //  break;
  //  ooppa.analyzeOnePix();

  //  ros::spinOnce();
  //  looprate.sleep();
  //}

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ooppa.analyzeOnePix();
  spinner.stop();
  ros::shutdown();
  return 0;
}