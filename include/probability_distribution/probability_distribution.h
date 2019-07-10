#ifndef PROBABILITY_DISTRIBUTION_H
#define PROBABILITY_DISTRIBUTION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

namespace probability_distribution
{
class ProbabilityDistribution
{
public: // constructors & destoructors
  ProbabilityDistribution();
  virtual ~ProbabilityDistribution(){}

public: // methods
  virtual Eigen::MatrixXd calcDencityFunction(const Eigen::MatrixXd input) = 0;
  virtual Eigen::MatrixXd calcLogDencityFunction(const Eigen::MatrixXd input) = 0;
  Eigen::MatrixXd genRandomValue();

private:

};
}  // namespace probability_distribution

#endif  // PROBABILITY_DISTRIBUTION_H
