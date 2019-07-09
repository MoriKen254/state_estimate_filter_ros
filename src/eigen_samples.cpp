#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

int main(void)
{
  MatrixXd a(1, 1);
  MatrixXd b(1, 1);
  MatrixXd x(1, 1);
  MatrixXd y(1, 1);
  x(0, 0) = 5.0;
  a(0, 0) = 2.0;
  b(0, 0) = 3.0;
  y = a * x + b;
  std::cout << y << std::endl;

  MatrixXd m(2, 2);
  m(0, 0) = 3;
  m(1, 0) = 2.5;
  m(0, 1) = -1;
  m(1, 1) = m(1, 0) + m(0, 1);
  std::cout << m << std::endl;
}
