#include <state_estimate_filter_ros/state_estimate_filter.h>
#include <state_estimate_filter_ros/particle_filter.h>
#include <matplotlibcpp/matplotlibcpp.h>

using state_estimate_filter_ros::StateEstimateFilter;
using state_estimate_filter_ros::ParticleFilter;
using Eigen::MatrixXd;

namespace plt = matplotlibcpp;

std::vector<double> genTrueh(const int step)
{
  std::vector<double> truth;

  for(int i = 0; i < step; ++i)
  {
    truth.push_back(static_cast<double>(i));
  }

  return truth;
}

std::vector<double> genObservation(std::vector<double>& truth)
{
  std::vector<double> observation;
  std::vector<double>::iterator itr = truth.begin();

  std::default_random_engine generator_observation;
  generator_observation.seed();
  std::normal_distribution<double> distribution_observation(0.0, 5.0);

  for(; itr != truth.end(); ++itr) {
    double noise = distribution_observation(generator_observation);
    observation.push_back(*itr + noise);
  }

  return observation;
}

void run_matplot(const std::vector<double> time,
                 const std::vector<double> truth, const std::vector<double> obs, const std::vector<double> estimate)
{
  plt::xlim(-10.0, 70.0);
  plt::ylim(-10.0, 70.0);
  plt::named_plot("true", time, truth, "r--");
  plt::named_plot("observation", time, obs, "b--");
  plt::named_plot("estimate", time, estimate, "g-");
  plt::legend();
  plt::show();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_estimate_filter");
  ros::NodeHandle nh;

  MatrixXd system_a(1, 1);
  MatrixXd system_b(1, 1);
  MatrixXd system_c(1, 1);
  system_a(0, 0) = 1.0;
  system_b(0, 0) = 0.0;
  system_c(0, 0) = 1.0;

  int step = 50;
  std::vector<double> truth = genTrueh(step);
  std::vector<double> observation = genObservation(truth);
  std::vector<double> time = truth;

  std::default_random_engine generator_observation;
  generator_observation.seed();
  std::normal_distribution<double> distribution_observation(0.0, 1.0);
  int num_particle = 100;
  MatrixXd vec_init_val(1, 1);
  vec_init_val(0, 0) = distribution_observation(generator_observation);
  MatrixXd vec_weight(1, 1);
  vec_weight(0, 0) = 1.0/static_cast<double>(num_particle);

  //particle_filter->initParticles(100, vec_init_val, vec_weight);
  StateEstimateFilter* particle_filter = new ParticleFilter(nh,
                                                            system_a, system_b, system_c,
                                                            num_particle, vec_init_val, vec_weight);
  particle_filter->setInitVal(vec_init_val);

  ros::Rate loop_rate(1000);

  MatrixXd input(1, 1);
  input(0, 0) = 0.0;

  std::vector<double> estimate;
  int step_curr = 0;
  while (ros::ok())
  {
    if(step_curr++ == step)
      break;

    particle_filter->estimate(input);
    estimate.push_back(particle_filter->vec_estimate_curr_(0, 0));
    ros::spinOnce();
    loop_rate.sleep();
  }

  run_matplot(time, truth, observation, estimate);

  return 0;
}
