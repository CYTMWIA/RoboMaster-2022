#include <iostream>

#include <toml.hpp>
#include <fmt/core.h>
#include <toml.hpp>

#include "config.hpp"
#include "logging.hpp"

namespace rmcv::config
{
    class Config::Impl
    {
    public:
        toml::value data_;
        std::string path_;

        std::vector<std::string> split_string(const std::string str, const char split_char = '.')
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
        T dot_find_or(std::string dotkeys, T fallback)
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

        template <typename T>
        void print_kv(std::string key, const T &value)
        {
            fmt::print("{}: {}\n", key, value);
        }

        template <typename T>
        void print_kv(std::string key, const std::vector<T> &vec)
        {
            print_kv(key, fmt::format("[{}]", fmt::join(vec, ", ")));
        }
    };

    Config::Config() : pimpl{std::make_unique<Impl>()} {}
    Config::~Config() = default;

    void Config::read(const std::string &path)
    {
        pimpl->path_ = path;
        pimpl->data_ = toml::parse(path);

#define READ_CONFIG(name, type, fallback)             \
    name = pimpl->dot_find_or<type>(#name, fallback); \
    pimpl->print_kv(#name, name);
        fmt::print("------------------------------------------------------------\n");

        READ_CONFIG(capture.target, std::string, "camera");

        READ_CONFIG(communicate.enable, toml::boolean, 0);

        READ_CONFIG(video.path, std::string, "");

        READ_CONFIG(image.path, std::string, "");

        READ_CONFIG(camera.id, toml::integer, 0);
        READ_CONFIG(camera.manufacturer, std::string, "");
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

        READ_CONFIG(serial.enable, toml::boolean, 0);
        READ_CONFIG(serial.port, std::string, "/dev/ttyUSB0");
        READ_CONFIG(serial.baud_rate, toml::integer, 115200);

        READ_CONFIG(vofa.enable, toml::boolean, 0);
        READ_CONFIG(vofa.ip, std::string, "192.168.137.1");
        READ_CONFIG(vofa.port, toml::integer, 1347);

        READ_CONFIG(debug.enable_window, toml::boolean, 0);

        fmt::print("------------------------------------------------------------\n");
#undef READ_CONFIG
    }
}