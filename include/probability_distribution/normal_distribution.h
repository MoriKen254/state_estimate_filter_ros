#ifndef NORMAL_DISTRIBUTION_H
#define NORMAL_DISTRIBUTION_H

#include <probability_distribution/probability_distribution.h>

namespace probability_distribution
{
class NormalDistribution : public ProbabilityDistribution
{
public: // constructors & destoructors
  NormalDistribution();

public: // methods
  Eigen::MatrixXd calcDencityFunction(const Eigen::MatrixXd input);
  Eigen::MatrixXd calcLogDencityFunction(const Eigen::MatrixXd input);

private:

};
}  // namespace probability_distribution

#endif  // PROBABILITY_DISTRIBUTION_H
