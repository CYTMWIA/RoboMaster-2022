#ifndef __WORK_THREAD_BASETHREAD_HPP__
#define __WORK_THREAD_BASETHREAD_HPP__

namespace rmcv::work_thread
{
    class BaseThread
    {
    public:
        virtual void up() = 0;
    };
}

#endif