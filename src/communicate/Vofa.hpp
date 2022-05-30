#ifndef __COMMUNICATE_VOFA_HPP__
#define __COMMUNICATE_VOFA_HPP__

#include <vector>
#include <string>
#include <cstring>

#include <boost/asio.hpp>

namespace rmcv::communicate
{
    class Vofa
    {
    private:
        boost::asio::io_context io_context_;
        boost::asio::ip::udp::socket udp_socket_;
        boost::asio::ip::udp::endpoint vofa_endpoint_;

    public:
        Vofa(std::string vofa_ip, uint16_t vofa_port);

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
}

#endif