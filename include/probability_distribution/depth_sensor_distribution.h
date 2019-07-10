#ifndef DEPTH_SENSOR_DISTRIBUTION_H
#define DEPTH_SENSOR_DISTRIBUTION_H

#include <probability_distribution/probability_distribution.h>

namespace probability_distribution
{
class DepthSensorDistribution : public ProbabilityDistribution
{
public: // constructors & destoructors
  DepthSensorDistribution();

public: // methods
  Eigen::MatrixXd calcDencityFunction(const Eigen::MatrixXd input);
  Eigen::MatrixXd calcLogDencityFunction(const Eigen::MatrixXd input);

private:

};
}  // namespace probability_distribution

#endif  // DEPTH_SENSOR_DISTRIBUTION_H
