#include <probability_distribution/normal_distribution.h>

using probability_distribution::NormalDistribution;
using Eigen::MatrixXd;

// Class methods definitions
NormalDistribution::NormalDistribution() : ProbabilityDistribution ()
{
}

Eigen::MatrixXd NormalDistribution::calcDencityFunction(const MatrixXd input)
{
  // result = 1/(sqrt(2π*σ^2)) * exp{-(x-μ)^2/(2σ^2) }
  // result = fact1 * fact3
  // fact1 = 1/(sqrt(2π*σ^2))
  // fact2 = -(x-μ)^2/(2σ^2)
  // fact3 = exp ( fact2 )
  // result = fact1 * fact3

  MatrixXd variance(1, 1);
  variance(0, 0) = 3.0;

}
