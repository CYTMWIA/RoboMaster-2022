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

        void rawdata(std::vector<uint8_t> data);

        template <typename T>
        void justfloat(std::vector<T> nums)
        {
            rawdata(to_justfloat(nums));
        }

        template <typename T>
        static std::vector<uint8_t> to_justfloat(std::vector<T> nums)
        {
            const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f};

            std::vector<float> res;
            res.reserve(nums.size() + 1);

            for (const T &n : nums)
                res.push_back(n);
            
            int fsize = sizeof(float) * nums.size();
            memcpy(((uint8_t*)res.data()) + fsize, tail, 4);

            return std::vector<uint8_t>{((uint8_t*)res.data()), ((uint8_t*)res.data())+fsize+4};
        }
    };
}

#endif