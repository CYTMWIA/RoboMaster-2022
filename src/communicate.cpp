#include <thread>
#include <chrono>
#include <iostream>

#include "logging/logging.hpp"
#include "threading/threading.hpp"
#include "communicate/communicate.hpp"

#include "communicate.hpp"

namespace rmcv
{
    using namespace config;
    using namespace threading;
    using namespace communicate;

    void thread_communicate(const Config &cfg)
    {
        Serial serial{cfg.serial.port, (uint32_t)cfg.serial.baud_rate};

        auto tick_start = std::chrono::steady_clock::now();
        auto interval = std::chrono::milliseconds{1000}/50;
        RobotStatus robot_status;
        CmdToEc output;
        output.pitch = output.yaw = 0;
        while (true)
        {
            // __LOG_DEBUG("{}", std::chrono::steady_clock::now()-tick_start);
            tick_start = std::chrono::steady_clock::now();

            if (RoslikeTopic<CmdToEc>::updated("cmd_to_ec"))
            {
                output = RoslikeTopic<CmdToEc>::get("cmd_to_ec", true);
            }
            else
            {
                // 衰减输出
                output.yaw *= 0.6;
                output.pitch *= 0.6;
            }

            serial.send(output);
            serial.update(robot_status);

            RoslikeTopic<RobotStatus>::set("robot_status", robot_status);

            std::this_thread::sleep_until(tick_start+interval);
        }
    }
}