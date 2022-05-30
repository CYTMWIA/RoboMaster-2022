#ifndef __COMMUNICATE_SERIAL_HPP__
#define __COMMUNICATE_SERIAL_HPP__

#include <string>
#include <vector>
#include <memory>

#include "Cmd.hpp"
#include "Vofa.hpp"

namespace rmcv::communicate
{
    class Serial
    {
    private:
        class Impl;
        std::unique_ptr<Impl> pimpl;

    public:
        Serial(std::string dev = "/dev/ttyUSB0", uint32_t baud_rate = 115200);
        ~Serial();

        void send(const CvStatus &status);
        void update(RobotStatus &status);
    };
}

#endif