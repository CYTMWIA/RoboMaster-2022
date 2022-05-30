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

    CommunicateThread::CommunicateThread(const rmcv::config::Config &cfg) : vofa_enable_(cfg.vofa.enable),
                                                                            vofa_ip_(cfg.vofa.ip),
                                                                            vofa_port_(cfg.vofa.port),
                                                                            serial_enable_(cfg.serial.enable),
                                                                            serial_port_(cfg.serial.port),
                                                                            serial_baud_rate_(cfg.serial.baud_rate)
    {
        freq_ = 100;
    }

    void CommunicateThread::run()
    {
        using namespace rmcv::communicate;
        using namespace rmcv::threading;

        std::unique_ptr<Vofa> pvofa;
        if (vofa_enable_) pvofa = std::make_unique<Vofa>(vofa_ip_, vofa_port_);

        std::unique_ptr<Serial> pserial;
        if (serial_enable_) pserial = std::make_unique<Serial>(serial_port_, serial_baud_rate_);

        RobotStatus robot_status;
        CmdToEc output;
        output.pitch = output.yaw = 0;

        cycle([&]()
        {
            if (vofa_enable_)
            {
                pvofa->justfloat(RoslikeTopic<std::vector<float>>::get("vofa_justfloat", true));
            }

            if (serial_enable_)
            {
                output = RoslikeTopic<CmdToEc>::get("cmd_to_ec", true);

                // if (output.pitch!=0 && output.yaw!=0)
                // __LOG_DEBUG("Sending Pitch {}, Yaw {}", output.pitch, output.yaw);
                pserial->send(output);

                pserial->update(robot_status);

                RoslikeTopic<RobotStatus>::set("robot_status", robot_status);
            } 
        });
    }
}