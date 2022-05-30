#include <iostream>

#include <toml.hpp>

#include "config.hpp"
#include "logging.hpp"

namespace rmcv::config
{
    void Config::read(const config_path &path)
    {
        path_ = path;
        data_ = toml::parse(path);

#define READ_CONFIG(name, type, fallback)      \
    name = dot_find_or<type>(#name, fallback); \
    std::cout << "> " << #name << ": " << name << std::endl;

        READ_CONFIG(capture.target, std::string, "");

        READ_CONFIG(communicate.enable, toml::boolean, 0);

        READ_CONFIG(video.path, std::string, "");

        READ_CONFIG(image.path, std::string, "");

        READ_CONFIG(camera.id, toml::integer, 0);
        READ_CONFIG(camera.exposure_time, toml::floating, 10000);
        READ_CONFIG(camera.gain, toml::floating, 0);
        READ_CONFIG(camera.white_balance_red, toml::floating, 1);
        READ_CONFIG(camera.white_balance_green, toml::floating, 1);
        READ_CONFIG(camera.white_balance_blue, toml::floating, 1);
        READ_CONFIG(camera.calibration_file, std::string, "");

        READ_CONFIG(model.bin_file, std::string, "");
        READ_CONFIG(model.xml_file, std::string, "");
        READ_CONFIG(model.onnx_file, std::string, "");

        READ_CONFIG(serial.port, std::string, "/dev/ttyUSB0");
        READ_CONFIG(serial.baud_rate, toml::integer, 115200);

        READ_CONFIG(debug.enable_window, toml::boolean, 0);

#undef READ_CONFIG
    }

    template <typename T>
    T Config::dot_find_or(std::string dotkeys, T fallback)
    {
        uint16_t key_start = 0, dot = 0;
        toml::value value = data_;
        while (dot < dotkeys.size())
        {
            while (dot < dotkeys.size() && dotkeys[dot] != '.')
                dot++;
            std::string key = dotkeys.substr(key_start, dot - key_start);
            if (dot >= dotkeys.size() && value.as_table().contains(key)) // last key
            {
                try
                {
                    return toml::find<T>(value, key);
                }
                catch (const std::exception &e)
                {
                    __LOG_WARNING("{} 参数错误，使用默认值({})\n{}", dotkeys, fallback, e.what());
                    break;
                }
            }
            else
            {
                try
                {
                    // 缺少std::move的话会导致段错误，使用临时变量也可以避免段错误
                    value = toml::find(std::move(value), key);
                }
                catch (const std::exception &e)
                {
                    break;
                }
                key_start = ++dot;
            }
        }
        return fallback;
    }
}