
/////////////////
// Provides a Gaussian kernel
// for weighted averaging
// and other stats functions
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//

#ifndef UNL_ROBOTICS_GAUSSKERNEL_H
#define UNL_ROBOTICS_GAUSSKERNEL_H

//std
#include <vector>
#include <utility>

namespace UNL_Robotics {

  typedef std::vector<double> kernel_row;
  typedef std::vector<kernel_row> kernel_type;

  template<typename dataT>
  std::pair<double, double> calcMeanAndStdDev(const std::vector<dataT>& v);
  
  double gaussian(double x, double mu, double sigma);

  //mu and sigma will default to kernelRadius and 1/2 kernelRadius, respectively, if not set
  kernel_type produce2dGaussianKernel(unsigned kernelRadius,
                                      double mu = -1.0, double sigma = -1.0);
  
  template<typename dataT>
  std::vector<dataT> truncate_n_return_middle_vals(std::vector<dataT> orig, unsigned n);
};

#endif

#include "gaussKernel.ipp"
