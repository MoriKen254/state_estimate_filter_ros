#ifndef NORMAL_DISTRIBUTION_H
#define NORMAL_DISTRIBUTION_H

#include <probability_distribution/probability_distribution.h>

namespace probability_distribution
{
class NormalDistribution : public ProbabilityDistribution
{
public: // constructors & destoructors
  NormalDistribution();
  NormalDistribution(const Eigen::MatrixXd mean, const Eigen::MatrixXd variance);

public: // methods
  Eigen::MatrixXd calcDencityFunction(const Eigen::MatrixXd input);
  Eigen::MatrixXd calcLogDencityFunction(const Eigen::MatrixXd input);

private:
  Eigen::MatrixXd mean_;
  Eigen::MatrixXd variance_;

};
}  // namespace probability_distribution

#endif  // PROBABILITY_DISTRIBUTION_H
