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
  MatrixXd varianceSqrt(1, 1);
  variance(0, 0) = 4.0;
  varianceSqrt = variance.cwiseSqrt().matrix();// array().abs().sqrt();
  MatrixXd fact1(1, 1);
  fact1 = 1 / (sqrt(2 * M_PI) * varianceSqrt.array());

  MatrixXd mean(1, 1);
  mean(0, 0) = 0.0;
  MatrixXd squeredError(1, 1);
  squeredError(0, 0) = std::pow(input(0, 0) - mean(0, 0), 2);

  MatrixXd fact2(1, 1);
  fact2(0, 0) = - squeredError(0, 0) / (2 * variance(0, 0));

  MatrixXd fact3(1, 1);
  fact3(0, 0) = std::exp(fact2(0, 0));

  MatrixXd result(1, 1);
  result(0, 0) = fact1(0, 0) * fact3(0, 0);

  return result;
}

Eigen::MatrixXd NormalDistribution::calcLogDencityFunction(const MatrixXd input)
{
  // TODO: 尤度関数をこれに切り替える
  // result = -log(sqrt(2π*σ^2)) - (x-μ)^2/(2σ^2)
  // result = fact1 + fact2
  // fact1 = -log(sqrt(2π)*σ)
  // fact2 = -(x-μ)^2/(2σ^2)　とおく
  // result = 1/(sqrt(2π*σ^2)) * exp{-(x-μ)^2/(2σ^2) }

  MatrixXd variance(1, 1);
  MatrixXd varianceSqrt(1, 1);
  variance(0, 0) = 4.0;
  varianceSqrt = variance.cwiseSqrt().matrix();// array().abs().sqrt();
  MatrixXd fact1(1, 1);
  fact1(0, 0) = - std::log(sqrt(2 * M_PI) * varianceSqrt(0, 0));

  // queredError = e^2
  MatrixXd mean(1, 1);
  mean(0, 0) = 0.0;
  MatrixXd squeredError(1, 1);
  squeredError(0, 0) = std::pow(input(0, 0) - mean(0, 0), 2);

  // fact2 = -(x-μ)^2/(2σ^2)
  MatrixXd fact2(1, 1);
  fact2(0, 0) = - squeredError(0, 0) / (2 * variance(0, 0));

  MatrixXd result(1, 1);
  result(0, 0) = fact1(0, 0) + fact2(0, 0);

  return result;
}
