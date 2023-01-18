
#include "gaussKernel.h"
//std
#include <cmath>

double UNL_Robotics::gaussian(double x, double mu, double sigma)
{
  const double a = ( x - mu ) / sigma;
  return std::exp( -0.5 * a * a );
}

UNL_Robotics::kernel_type UNL_Robotics::produce2dGaussianKernel(unsigned kernelRadius,
                                                                double mu, double sigma)
{
  if(mu == -1.0)
    mu = kernelRadius;
  if(sigma == -1.0)
    sigma = kernelRadius / 2.0;
  kernel_type kernel2d(2*kernelRadius+1, kernel_row(2*kernelRadius+1));
  double sum = 0.0;
  // compute values
  for (unsigned row = 0; row < kernel2d.size(); ++row) {
    for (unsigned col = 0; col < kernel2d[row].size(); ++col) {
      double x = gaussian(row, mu, sigma) * gaussian(col, mu, sigma);
      kernel2d[row][col] = x;
      sum += x;
    }
  }
  
  // normalize
  for (unsigned row = 0; row < kernel2d.size(); ++row)
    for (unsigned col = 0; col < kernel2d[row].size(); ++col)
      kernel2d[row][col] /= sum;
  
  return kernel2d;
}
