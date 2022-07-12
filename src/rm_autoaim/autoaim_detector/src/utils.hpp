#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <algorithm>
#include <cmath>
#include <vector>

namespace rm_autoaim
{

template <typename T, typename Pred>
void filter_emplace(std::vector<T>& vec, Pred pred_keep)
{
  vec.erase(
      std::remove_if(vec.begin(), vec.end(), [pred_keep](T& e) { return !pred_keep(e); }),
      vec.end());
}

template <typename T, typename Pred>
std::vector<T> filter_copy(const std::vector<T>& vec, Pred pred_keep)
{
  std::vector<T> new_vec;
  std::copy_if(vec.begin(), vec.end(), std::back_inserter(new_vec), pred_keep);
  return std::move(new_vec);
}

inline double to_angle(double rad) { return rad * (180.0 / M_PI); }
inline double to_rad(double angle) { return angle * (M_PI / 180.0); }

}  // namespace rm_autoaim

#endif