#include "planner/timer.h"

Timer::Timer()
{
    start_t_ = std::chrono::high_resolution_clock::now();
    accum_t_ = std::chrono::duration<long double>(0);
    stopped_ = true;
    paused_ = false;
}

Timer::~Timer()
{

}

void Timer::start()
{
    if (stopped_)
    {
        stopped_ = false;
        paused_ = false;
        start_t_ = std::chrono::high_resolution_clock::now();
        accum_t_ = std::chrono::duration<long double>(0);
    }
    else if (paused_)
    {
        paused_ = false;
        start_t_ = std::chrono::high_resolution_clock::now();
    }
}

void Timer::pause()
{
    if (!paused_ && !stopped_)
    {
        paused_ = true;
        accum_t_ += std::chrono::duration_cast<std::chrono::duration<long double>>(std::chrono::high_resolution_clock::now() - start_t_);
    }
}

void Timer::reset()
{
    if (!stopped_)
    {
        start_t_ = std::chrono::high_resolution_clock::now();
        accum_t_ = std::chrono::duration<long double>(0);
        stopped_ = true;
        paused_ = false;
    }
}

int64_t Timer::elapsedMillis()
{
    if (!stopped_)
    {
        if (!paused_)
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(accum_t_ + (std::chrono::high_resolution_clock::now() - start_t_)).count();
        }
        else
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(accum_t_).count();
        }
    }
    else
    {
        return std::chrono::duration<int64_t>(0).count();
    }
}
