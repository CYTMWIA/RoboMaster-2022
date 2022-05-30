#include <iostream>

#include <toml.hpp>
#include <fmt/core.h>

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
    print_kv(#name, name);
        fmt::print("------------------------------------------------------------\n");

        READ_CONFIG(capture.target, std::string, "camera");

        READ_CONFIG(communicate.enable, toml::boolean, 0);
        READ_CONFIG(communicate.target, std::string, "serial");

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

        READ_CONFIG(ekf.q, std::vector<double>, std::vector<double>({1, 1, 1, 1, 1, 1}));
        READ_CONFIG(ekf.r, std::vector<double>, std::vector<double>({1, 1, 1, 1, 1, 1}));

        READ_CONFIG(serial.port, std::string, "/dev/ttyUSB0");
        READ_CONFIG(serial.baud_rate, toml::integer, 115200);

        READ_CONFIG(vofa.ip, std::string, "192.168.137.1");
        READ_CONFIG(vofa.port, toml::integer, 1347);

        READ_CONFIG(debug.enable_window, toml::boolean, 0);

        fmt::print("------------------------------------------------------------\n");
#undef READ_CONFIG
    }

    template <typename T>
    T Config::dot_find_or(std::string dotkeys, T fallback)
    {
        auto keys = split_string(dotkeys, '.');
        toml::value value = data_;
        for (int i = 0; i < keys.size(); i++)
        {
            std::string key = keys[i];
            try
            {
                value = toml::find(std::move(value), key);
            }
            catch (const std::exception &e)
            {
                break;
            }
            if (i == keys.size() - 1)
                return toml::get<T>(value);
        }
        return fallback;
    }

    std::vector<std::string> Config::split_string(const std::string str, const char split_char)
    {
        std::vector<std::string> res;
        int begin = 0;
        for (int end = 0; end < str.size(); end++)
        {
            if (str[end] == split_char)
            {
                res.push_back(str.substr(begin, end));
                begin = end + 1;
            }
        }
        if (begin < str.size())
            res.push_back(str.substr(begin));
        return res;
    }

    template <typename T>
    void Config::print_kv(std::string key, const T &value)
    {
        fmt::print("{}: {}\n", key, value);
    }

    template <typename T>
    void Config::print_kv(std::string key, const std::vector<T> &vec)
    {
        print_kv(key, fmt::format("[{}]", fmt::join(vec, ", ")));
    }
}