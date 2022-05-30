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

    void cycle(int freq, const std::function<void(void)> &func)
    {
        auto tick_start = std::chrono::steady_clock::now();
        auto interval = std::chrono::milliseconds{1000} / freq;
        while (true)
        {
            tick_start = std::chrono::steady_clock::now();

            func();

            std::this_thread::sleep_until(tick_start + interval);
        }
    }

    void communicate_serial(const Config &cfg)
    {
        Serial serial{cfg.serial.port, (uint32_t)cfg.serial.baud_rate};

        RobotStatus robot_status;
        CmdToEc output;
        output.pitch = output.yaw = 0;

        cycle(100, [&]()
        {
            output = RoslikeTopic<CmdToEc>::get("cmd_to_ec", true);

            // if (output.pitch!=0 && output.yaw!=0)
            //     __LOG_DEBUG("Sending Pitch {}, Yaw {}", output.pitch, output.yaw);
            serial.send(output);

            serial.update(robot_status);

            RoslikeTopic<RobotStatus>::set("robot_status", robot_status);
        });
    }

    void communicate_vofa(const Config &cfg)
    {
        Vofa vofa{cfg.vofa.ip, (uint16_t)cfg.vofa.port};

        cycle(100, [&]()
        {
            vofa.justfloat(RoslikeTopic<std::vector<float>>::get("vofa_justfloat", true));
        });
    }

    void thread_communicate(const Config &cfg)
    {
        if (cfg.communicate.target == "serial")
        {
            communicate_serial(cfg);
        }
        else if (cfg.communicate.target == "vofa")
        {
            communicate_vofa(cfg);
        }
    }
}