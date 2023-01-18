
//Implementation file

#include <cmath>
#include <algorithm>
#include <numeric>

template<typename dataT>
std::pair<double, double> UNL_Robotics::calcMeanAndStdDev(const std::vector<dataT>& v)
{
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();
  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / v.size());
  return std::pair<double, double>(mean, stdev);
}

//Truncate the highest and lowest "n" values and return the remaining middle values
template<typename dataT>
std::vector<dataT> UNL_Robotics::truncate_n_return_middle_vals(std::vector<dataT> orig, unsigned n)
{
  std::sort(orig.begin(), orig.end());
  return std::vector<dataT>(orig.begin()+n, orig.end()-n);
}  

