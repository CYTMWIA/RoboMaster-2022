#include "CommunicateThread.hpp"

#include "threading/threading.hpp"

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
        target_ = cfg.communicate.target;
        freq_ = 100;

        if (target_ == "serial")
        {
            target_func_ = std::bind(&CommunicateThread::serial, this, cfg.serial.port, cfg.serial.baud_rate);
        }
        else if (target_ == "vofa")
        {
            target_func_ = std::bind(&CommunicateThread::vofa, this, cfg.vofa.ip, cfg.vofa.port);
        }
    }

    void CommunicateThread::vofa(const std::string ip, const uint16_t port)
    {
        using namespace rmcv::communicate;
        using namespace rmcv::threading;

        Vofa vofa{ip, port};

        cycle([&]()
        {
            vofa.justfloat(RoslikeTopic<std::vector<float>>::get("vofa_justfloat", true));
        });
    }

    void CommunicateThread::serial(const std::string port, const uint32_t baud_rate)
    {
        using namespace rmcv::communicate;
        using namespace rmcv::threading;

        Serial serial{port, baud_rate};

        RobotStatus robot_status;
        CmdToEc output;
        output.pitch = output.yaw = 0;

        cycle([&]()
        {
            output = RoslikeTopic<CmdToEc>::get("cmd_to_ec", true);

            // if (output.pitch!=0 && output.yaw!=0)
            //     __LOG_DEBUG("Sending Pitch {}, Yaw {}", output.pitch, output.yaw);
            serial.send(output);

            serial.update(robot_status);

            RoslikeTopic<RobotStatus>::set("robot_status", robot_status);
        });
    }

    void CommunicateThread::up()
    {
        thread_ = std::thread(target_func_);
    }
}