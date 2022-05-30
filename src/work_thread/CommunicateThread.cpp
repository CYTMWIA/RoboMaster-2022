#include "CommunicateThread.hpp"

#include "threading/threading.hpp"
#include "logging/logging.hpp"

namespace rmcv::work_thread
{
    void CommunicateThread::cycle(const std::function<void(void)> &func)
    {
        auto tick_start = std::chrono::steady_clock::now();
        auto interval = std::chrono::milliseconds{1000} / freq_;
        while (true)
        {
            tick_start = std::chrono::steady_clock::now();

            func();

            std::this_thread::sleep_until(tick_start + interval);
        }
    }

    CommunicateThread::CommunicateThread(const rmcv::config::Config &cfg)
    {
        freq_ = 100;

        vofa_ip_ = cfg.vofa.ip;
        vofa_port_ = cfg.vofa.port;
        serial_port_ = cfg.serial.port;
        serial_baud_rate_ = cfg.serial.baud_rate;
    }

    void CommunicateThread::run()
    {
        using namespace rmcv::communicate;
        using namespace rmcv::threading;

        Vofa vofa{vofa_ip_, vofa_port_};

        Serial serial{serial_port_, serial_baud_rate_};

        RobotStatus robot_status;
        CmdToEc output;
        output.pitch = output.yaw = 0;

        cycle([&]()
        {
            // Vofa
            vofa.justfloat(RoslikeTopic<std::vector<float>>::get("vofa_justfloat", true));


            // Serial

            output = RoslikeTopic<CmdToEc>::get("cmd_to_ec", true);

            // if (output.pitch!=0 && output.yaw!=0)
            // __LOG_DEBUG("Sending Pitch {}, Yaw {}", output.pitch, output.yaw);
            serial.send(output);

            serial.update(robot_status);

            RoslikeTopic<RobotStatus>::set("robot_status", robot_status);
        });
    }

    void CommunicateThread::up()
    {
        auto running = std::bind(&CommunicateThread::run, this);
        thread_ = std::thread(running);
    }
}