#include "serial.hpp"

namespace rmcv::io
{
    Serial::Serial(std::string dev, uint32_t baud_rate):
        serial_(dev, baud_rate)
    { }

    void Serial::write(const Message &content)
    {
        serial_.write(content);
    }
}