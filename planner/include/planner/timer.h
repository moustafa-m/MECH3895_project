#pragma once

#include <chrono>

class Timer
{
public:
    Timer();
    ~Timer();

    void start();
    void pause();
    void reset();
    int64_t elapsedMillis();

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_t_;
    std::chrono::duration<long double> accum_t_;

    bool stopped_;
    bool paused_;
};
