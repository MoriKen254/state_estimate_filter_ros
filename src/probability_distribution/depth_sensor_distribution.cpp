#include <probability_distribution/normal_distribution.h>
#include <probability_distribution/depth_sensor_distribution.h>

using probability_distribution::NormalDistribution;
using probability_distribution::DepthSensorDistribution;
using Eigen::MatrixXd;

// Class methods definitions
DepthSensorDistribution::DepthSensorDistribution() : ProbabilityDistribution ()
{
}

Eigen::MatrixXd DepthSensorDistribution::calcDencityFunction(const MatrixXd input)
{
  ProbabilityDistribution* prob_dis_center = new NormalDistribution();
  ProbabilityDistribution* prob_dis_front = new NormalDistribution();
  ProbabilityDistribution* prob_dis_back = new NormalDistribution();

  //MatrixXd input(1, 1);
  //input(0, 0) = 1.5;
  MatrixXd output_center = prob_dis_center->calcDencityFunction(input);
  MatrixXd output_front = prob_dis_front->calcDencityFunction(input);
  MatrixXd output_back = prob_dis_back->calcDencityFunction(input);

  MatrixXd result = output_center + output_front + output_back;

  return result;
}

Eigen::MatrixXd DepthSensorDistribution::calcLogDencityFunction(const MatrixXd input)
{
  ProbabilityDistribution* prob_dis_center = new NormalDistribution();
  ProbabilityDistribution* prob_dis_front = new NormalDistribution();
  ProbabilityDistribution* prob_dis_back = new NormalDistribution();

  //MatrixXd input(1, 1);
  //input(0, 0) = 1.5;
  MatrixXd output_center = prob_dis_center->calcLogDencityFunction(input);
  MatrixXd output_front = prob_dis_front->calcLogDencityFunction(input);
  MatrixXd output_back = prob_dis_back->calcLogDencityFunction(input);

  MatrixXd result = output_center + output_front + output_back;
  return result;
}
