#include "utils.h"

namespace rmcv
{
    void makesure_file_exist(std::string path)
    {
        if (path == "")
            __LOG_ERROR_AND_EXIT("路径为空");
        
        if (!std::filesystem::exists(path))
            __LOG_ERROR_AND_EXIT("路径不存在");
        
        if (std::filesystem::is_directory(path))
            __LOG_ERROR_AND_EXIT("路径实际指向了目录");
    }
}