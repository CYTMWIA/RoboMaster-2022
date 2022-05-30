#include <bit>
#include <iostream>

#include "serial/serial.h"

#include "config.hpp"
#include "communicate.hpp"
#include "logging.hpp"

namespace rmcv
{
    void start_communicate(const Config::Communicate& cfg)
    {
        serial::Serial ser{cfg.port, uint32_t(cfg.baud_rate)};

        while (1)
        {
            auto s = VariableCenter<std::vector<uint8_t>>::get("send2ec");
            // for (int i=0;i<s.size();i++)
            //     std::cout << std::to_string(s[i]) << " ";
            // std::cout << std::endl;
            ser.write(s);
        }
    }
}