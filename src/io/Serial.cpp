#include "Serial.hpp"

namespace rmcv::io
{
    Serial::Serial(std::string dev, uint32_t baud_rate) : serial_(dev, baud_rate), recv_buffer_(kCmdToCvSize * 2)
    {
    }

    void Serial::send(const CvStatus &status)
    {
        uint8_t data[kCmdToEcSize];
        CmdToEc_make(&status, data);
        serial_.write(data, kCmdToEcSize);
    }

    void Serial::update(RobotStatus &status)
    {
        auto new_bytes_count = serial_.available();
        if (new_bytes_count == 0)
            return;

        serial_.read(recv_buffer_, new_bytes_count);

        auto res = CmdToCv_parse(&status, recv_buffer_.data(), recv_buffer_.size());
        if (res <= 0)
            recv_buffer_.clear();
        else
            recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin()+res);
    }
}