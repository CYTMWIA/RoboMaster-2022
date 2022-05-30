#ifndef __COMMUNICATE_SERIAL_HPP__
#define __COMMUNICATE_SERIAL_HPP__

#include <string>
#include <vector>

#include "serial/serial.h"

#include "Cmd.hpp"
#include "Vofa.hpp"

namespace rmcv::communicate
{
    class Serial
    {
    private:
        serial::Serial serial_;
        std::vector<uint8_t> recv_buffer_;

        
    public:
        Serial(std::string dev = "/dev/ttyUSB0", uint32_t baud_rate = 115200);

        void send(const CvStatus &status);
        void update(RobotStatus& status);

        template <typename... Args>
        void vofa_justfloat(Args &&...nums)
        {
            std::vector<uint8_t> data = Vofa::to_justfloat({nums...});
            serial_.write(data);
        }
    };
}

#endif