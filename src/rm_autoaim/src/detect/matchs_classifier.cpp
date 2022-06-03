#include "matchs_classifier.hpp"

#include <algorithm>
#include <vector>

namespace rm_autoaim
{
void MatchsClassifier::add_match(int32_t id, HammingMatch match) { matchs[id] = match; }

int32_t MatchsClassifier::operator()(const cv::Mat &src, double thresh)
{
  if (matchs.size() <= 0) return -1;

  std::vector<std::tuple<int32_t, double>> id2conf;
  for (const auto &mp : matchs)
    id2conf.push_back(std::tuple<int32_t, double>(mp.first, mp.second(src)));
  std::sort(id2conf.begin(), id2conf.end(),
            [](auto &p1, auto &p2) { return std::get<1>(p2) < std::get<1>(p1); });  // 降序

  // for (int i=0;i<id2conf.size();i++)
  //     std::cout << std::get<0>(id2conf[i]) << " " << std::get<1>(id2conf[i]) << " | ";
  // std::cout << std::endl;

  if (std::get<1>(id2conf[0]) >= thresh)
    return std::get<0>(id2conf[0]);
  else
    return -1;
}
}  // namespace rm_autoaim
