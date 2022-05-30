#include "logging/logging.hpp"

#include "MessageBus.hpp"

namespace rmcv::threading
{
    Message::Message(MessageContent content_, ThreadCode from_)
        :content(content_), from(from_)
    {}

    MessageBus& MessageBus::connect()
    {
        // 为什么这里和声明相比缺少了static
        // https://stackoverflow.com/questions/31305717/member-function-with-static-linkage

        // Meyers' Singleton, works in C++11
        static MessageBus instance;
        return instance;
    }

    void MessageBus::checkin(ThreadCode code)
    {
        std::unique_lock<std::shared_mutex> xlck(to_code_mu_);
        to_code_[std::this_thread::get_id()] = code;
    }

    bool MessageBus::is_checkin()
    {
        std::shared_lock<std::shared_mutex> slck(to_code_mu_);
        return to_code_.contains(std::this_thread::get_id());
    }

    ThreadCode MessageBus::get_code()
    {
        if (is_checkin())
        {
            std::shared_lock<std::shared_mutex> slck(to_code_mu_);
            return to_code_[std::this_thread::get_id()];
        }
        else
        {
            __LOG_WARNING("线程未登记");
            throw std::runtime_error("Thread Not Checkin Yet!");
        }
    }

    void MessageBus::send_message(ThreadCode to, MessageContent content)
    {
        ThreadCode code = get_code();
        Message msg{content, code};

        std::unique_lock<std::shared_mutex> xlck(msgbox_mu_[(int)to]);
        msgbox_[(int)to].push(msg);
    }

    bool MessageBus::has_message()
    {
        int code = (int)get_code();
        std::shared_lock<std::shared_mutex> slck(msgbox_mu_[code]);
        return !msgbox_[code].empty();
    }

    Message MessageBus::receive_message()
    {
        if (!has_message())
            return Message{MessageContent{}, (ThreadCode)(-1)};

        int code = (int)get_code();
        std::unique_lock<std::shared_mutex> xlck(msgbox_mu_[code]);
        Message msg = msgbox_[code].front();
        msgbox_[code].pop();
        return msg;
    }
}
