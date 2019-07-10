#include <gtest/gtest.h>
#include <probability_distribution/normal_distribution.h>

using probability_distribution::ProbabilityDistribution;
using probability_distribution::NormalDistribution;
using Eigen::MatrixXd;

const double EPS = 0.01;

// TEST CASES
TEST(ProbabilityDistributionTest, testNormal)
{
  ProbabilityDistribution* prob_dis = new NormalDistribution();

  MatrixXd input(1, 1);
  input(0, 0) = 1.5;
  MatrixXd output = prob_dis->calcDencityFunction(input);

  double result = output(0, 0);

  const double ANSWER = 0.15056871607740221;
  EXPECT_NEAR(result, ANSWER, EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "probability_distribution_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return ret;
}
