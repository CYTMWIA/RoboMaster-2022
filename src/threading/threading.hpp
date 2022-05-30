#ifndef __THREADING_HPP__
#define __THREADING_HPP__

#include <algorithm>
#include <condition_variable>
#include <shared_mutex>
#include <thread>
#include <vector>
#include <queue>
#include <unordered_map>
#include <iostream>

#include "logging.hpp"

namespace rmcv
{
    enum class ThreadCode
    {
        main = 0,
        capture,
        communicate,
        detect,
        record,
        strategy,
        web,
    };

    const int32_t MAX_THREADS = 8;

    class MessageContent: public std::vector<uint8_t>
    {
        /* 此class用于线程间通信、命令
         * 由于不涉及跨设备通信，所以不考虑大小端
         */
    public:
        template<typename T> void write(T&& val)
        {
            int count = size();
            resize(size() + sizeof(T));
            memcpy(data() + count, &val, sizeof(T));
        }
        template<typename T> T read()
        {
            if (read_pos_ >= size())
                throw std::runtime_error("MessageContent Read() 越界");

            T res;
            memcpy(&res, data() + read_pos_, sizeof(T));
            read_pos_ += sizeof(T);
            return res;
        }
    private:
        int32_t read_pos_ = 0;
    };

    struct Message
    {
        Message(MessageContent content_, ThreadCode from_);

        ThreadCode from;
        MessageContent content;
    };

    class MessageBus
    {
        /* 作为消息（命令）总线，该class使用单例模式
         * 使用connect来“连接”到总线
         */
    public:
        static MessageBus& connect();

        void checkin(ThreadCode code); // 在发送或接收消息之前，线程必须先在此“登记”自己的身份
        bool is_checkin();
        ThreadCode get_code();

        void send_message(ThreadCode to, MessageContent content);
        bool has_message();
        // void wait_message_for(const std::chrono::duration<Rep, Period>& duration);
        Message receive_message();
    private:
        MessageBus(){};

        std::unordered_map<std::thread::id, ThreadCode> to_code_;
        std::shared_mutex to_code_mu_;

        std::queue<Message> msgbox_[MAX_THREADS];
        std::shared_mutex msgbox_mu_[MAX_THREADS];
    };

    template <typename DataType>
    class SharedVariable
    {
        /* 模板类声明与实现写在一起
         * https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
         * 之前为了加速编译时，声明与实现是分离的
         * 导致每种类型的SharedVariable都需要手动声明，且在编写cmake文件时需要增加许多额外依赖
         * 所以最后为了方便，牺牲了编译时间，将声明与实现写在一起
         */
    public:
        SharedVariable()
        {
            ptr_input_ = &(data_[0]);
            ptr_output_ = &(data_[1]);
            std::fill(updated_, updated_ + MAX_THREADS, false);
        };

        template<typename T=DataType>
        void set(T&& data)
        {
            // 因为仅有一个写者，所以在写区时无需额外加锁
            *ptr_input_ = std::forward<T>(data);

            // 读写区互换
            std::unique_lock<std::shared_mutex> xlck(mu_read_); // 排它锁，其它线程不可读写
            std::swap(ptr_output_, ptr_input_);
            std::fill(updated_, updated_ + thread_count_, true);
            xlck.unlock();
            // 更新通知
            cv_update_.notify_all();
        };

        DataType get(bool allow_old_data=false)
        {
            int tid = get_this_thread_idx();
            
            std::shared_lock<std::shared_mutex> slck(mu_read_);
            if (!allow_old_data && !updated_[tid])
                cv_update_.wait(slck, [&]{ return updated_[tid]; });

            updated_[tid] = false;
            return *ptr_output_;
        };

    private:
        DataType *ptr_output_;
        DataType *ptr_input_;
        DataType data_[2];
        std::shared_mutex mu_read_;

        std::condition_variable_any cv_update_;

        uint8_t thread_count_ = 0;
        std::thread::id thread_list_[MAX_THREADS];
        bool updated_[MAX_THREADS];

        int get_this_thread_idx()
        {
            /* 这个函数看起来可以采用分离式编译，毕竟没看到有模板内容，但实际是不行的
             * 因为在分离时，该函数在实现部分中的写法如下
             *   template<class DataType>
             *   int SharedVariable<DataType>::get_this_thread_idx() { ... }
             * 所以实际上模板还是存在的。不采用分离式编译的详细理由可见此class头部的注释
             */

            auto this_id = std::this_thread::get_id();
            // 在已有线程id中查找
            for (int i = 0; i < thread_count_; i++)
            {
                if (thread_list_[i] == this_id)
                    return i;
            }
            // 找不到则添加
            thread_list_[thread_count_] = this_id;
            thread_count_ = (thread_count_ + 1) % MAX_THREADS;
            return thread_count_ - 1;
        };
    };

    template <typename DataType>
    class VariableCenter
    {
    private:
        static VariableCenter<DataType> instance_;
        VariableCenter(){};

        /* 重哈希会导致所有迭代器被非法化，即数据转移，这在其他线程读取数据时非常危险
         * 但是，重哈希仅若新元素数量大于 max_load_factor()*bucket_count() 才发生
         * 所以在此预留足够的bucket_count来避免重哈希，以此实现无锁使用map
         */
        std::unordered_map<std::string, SharedVariable<DataType>> varmap_{MAX_THREADS*8};
        // std::shared_mutex varmap_mu_;
    
    public:
        static bool exist(const std::string& name)
        {
            return instance_.varmap_.contains(name);
        };

        template<typename T=DataType>
        static void set(std::string name, T&& var)
        {
            // __LOG_DEBUG("Set {}", name);

            instance_.varmap_[name].set(std::forward<T>(var));
            
        };

        template<typename... Args>
        static DataType get(std::string name, Args&&... args)
        {
            // __LOG_DEBUG("Get {}", name);
            // 变长参数转发
            // https://stackoverflow.com/questions/16447951/how-to-forward-variable-number-of-arguments-to-another-function
            return instance_.varmap_[name].get(std::forward<Args>(args)...);
        };
    };
    template <typename T>
    VariableCenter<T> VariableCenter<T>::instance_{};
}

#endif