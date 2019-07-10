#include <probability_distribution/normal_distribution.h>

using probability_distribution::ProbabilityDistribution;
using probability_distribution::NormalDistribution;
using Eigen::MatrixXd;

// TEST CASES
double run(void)
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "probability_distribution_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  double ret = run();
  std::cout << ret << std::endl;
  spinner.stop();
  ros::shutdown();

  return 0;
}

