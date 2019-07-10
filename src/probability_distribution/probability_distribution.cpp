#include <probability_distribution/probability_distribution.h>

using probability_distribution::ProbabilityDistribution;

// Class methods definitions
ProbabilityDistribution::ProbabilityDistribution()
{
}

Eigen::MatrixXd ProbabilityDistribution::genRandomValue()
{
  std::random_device seed;
  std::mt19937 engine(seed());            // Mersenne twister

  std::uniform_real_distribution<double> dist(-100.0, 100.0);

  for ( int i = 0 ; i != 10 ; ++i )
    std::cout << dist(engine) << std::endl;
}
