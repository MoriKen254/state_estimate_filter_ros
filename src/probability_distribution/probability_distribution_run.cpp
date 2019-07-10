#include <probability_distribution/normal_distribution.h>
#include <probability_distribution/depth_sensor_distribution.h>
#include <matplotlibcpp/matplotlibcpp.h>

using probability_distribution::ProbabilityDistribution;
using probability_distribution::NormalDistribution;
using probability_distribution::DepthSensorDistribution;
using Eigen::MatrixXd;

namespace plt = matplotlibcpp;

// TEST CASES
double run_normal(void)
{
  ProbabilityDistribution* prob_dis = new NormalDistribution();

  MatrixXd input(1, 1);
  input(0, 0) = 1.5;
  MatrixXd output = prob_dis->calcDencityFunction(input);

  double result = output(0, 0);

  //const double ANSWER = 1.0;
  //EXPECT_EQ(result, ANSWER);
  return result;
}

double run_depth(void)
{
  ProbabilityDistribution* prob_dis = new DepthSensorDistribution();

  MatrixXd input(1, 1);
  input(0, 0) = 1.5;
  MatrixXd output = prob_dis->calcDencityFunction(input);

  double result = output(0, 0);

  //const double ANSWER = 1.0;
  //EXPECT_EQ(result, ANSWER);
  return result;
}

void run_matplot(void)
{
  int x[4] = {1,2,3,4};
  int y[4] = {1,3,2,4};
  plt::plot(x, y, "r--");
  plt::show();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "probability_distribution_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  double ret_normal = run_normal();
  double ret_depth = run_depth();
  run_matplot();
  std::cout << ret_normal << std::endl;
  std::cout << ret_depth << std::endl;
  spinner.stop();
  ros::shutdown();

  return 0;
}

