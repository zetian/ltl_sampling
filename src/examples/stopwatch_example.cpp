#include <iostream>
#include <thread>

#include "stopwatch/stopwatch.h"

int main() 
{
    stopwatch::StopWatch stopwatch;

    stopwatch.tic();

    const auto timer = stopwatch::make_timer(std::chrono::seconds(1));
    while (!timer.done()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto duration = stopwatch.toc();

    std::cout << "time elapsed (s): " << duration << std::endl;
    std::cout << "s, ms, us, ns " << stopwatch.stoc() << " , "
        << stopwatch.mtoc() << " , " << stopwatch.utoc() << " , " << stopwatch.ntoc() << std::endl;

    const auto cycles = stopwatch::time([] {
    for (std::size_t i = 0; i < 10; ++i) {
      std::cout << i << std::endl;
    }
    });

    std::cout << "To print out 10 numbers, it took " << cycles.count()
            << " cycles." << std::endl;

    return 0;
}