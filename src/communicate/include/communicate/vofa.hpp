#ifndef __COMMUNICATE_VOFA_HPP__
#define __COMMUNICATE_VOFA_HPP__

#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace rm_communicate
{
class Vofa
{
 private:
  class Impl;
  std::unique_ptr<Impl> pimpl;

 public:
  Vofa(std::string vofa_ip, uint16_t vofa_port);
  ~Vofa();

  void vofa_endpoint(std::string vofa_ip, uint16_t vofa_port);

  void rawdata(const std::vector<uint8_t> &data);

  template <typename T>
  void justfloat(const std::vector<T> &nums)
  {
    rawdata(to_justfloat(nums));
  }

  template <typename T>
  static std::vector<uint8_t> to_justfloat(const std::vector<T> &nums)
  {
    const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f};

    std::vector<float> fvec{nums.begin(), nums.end()};
    fvec.reserve(nums.size() + 1);

    uint8_t *const ptr = (uint8_t *)fvec.data();
    int size = sizeof(float) * nums.size();
    memcpy(ptr + size, tail, 4);

    return std::vector<uint8_t>{ptr, ptr + size + 4};
  }
};
}  // namespace rm_communicate

#endif