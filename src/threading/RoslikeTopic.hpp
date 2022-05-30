#ifndef __ROSLIKETOPIC_HPP__
#define __ROSLIKETOPIC_HPP__

#include <unordered_map>

#include "SharedVariable.hpp"

namespace rmcv::threading
{
    template <typename DataType>
    class RoslikeTopic
    {
    private:
        static RoslikeTopic<DataType> instance_;
        RoslikeTopic(){};

        /* 重哈希会导致所有迭代器被非法化，即数据转移，这在其他线程读取数据时非常危险
         * 但是，重哈希仅若新元素数量大于 max_load_factor()*bucket_count() 才发生
         * 所以在此预留足够的bucket_count来避免重哈希，以此实现无锁使用map
         */
        std::unordered_map<std::string, SharedVariable<DataType>> varmap_{MAX_THREADS * 8};
        // std::shared_mutex varmap_mu_;

    public:
        static bool exist(const std::string &name)
        {
            return instance_.varmap_.contains(name);
        };

        template <typename T = DataType>
        static void set(std::string name, T &&var)
        {
            // __LOG_DEBUG("Set {}", name);

            instance_.varmap_[name].set(std::forward<T>(var));
        };

        template <typename... Args>
        static DataType get(std::string name, Args &&...args)
        {
            // __LOG_DEBUG("Get {}", name);
            // 变长参数转发
            // https://stackoverflow.com/questions/16447951/how-to-forward-variable-number-of-arguments-to-another-function
            return instance_.varmap_[name].get(std::forward<Args>(args)...);
        };
    };
    template <typename T>

    RoslikeTopic<T> RoslikeTopic<T>::instance_{};
}

#endif