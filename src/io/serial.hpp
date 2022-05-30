#ifndef __RMCV_SERIAL_HPP__
#define __RMCV_SERIAL_HPP__

#include <string>
#include <vector>

#include "serial/serial.h"

namespace rmcv::io
{
    class Serial
    {
    using Message = std::vector<uint8_t>;

    private:
        serial::Serial serial_;
    public:
        Serial(std::string dev="/dev/ttyUSB0", uint32_t baud_rate=115200);

        void write(const Message &content);
        bool has_message();
        Message read();

        template<typename ...Args>
        void vofa_justfloat(Args&& ...nums)
        {
            const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f};
            std::vector<float> fs = {nums..., 0};
            memcpy(&fs[sizeof...(nums)], tail, 4); 
            serial_.write((uint8_t*)(fs.data()), sizeof(float)*(sizeof...(nums))+4);
        }
    };
}

#endif