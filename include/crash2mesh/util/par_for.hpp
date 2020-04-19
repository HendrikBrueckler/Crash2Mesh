#ifndef C2M_PAR_FOR_HPP
#define C2M_PAR_FOR_HPP

#include <crash2mesh/core/types.hpp>

#include <thread>
#if defined(C2M_PARALLEL)
#if 0 //defined(__cpp_lib_parallel_algorithm)
#include <algorithm>
#include <execution>
#elif 0
#include <future>
#include <atomic>
#else
#include <sandbox/ThreadPool/ThreadPool.h>
#endif
#endif

namespace c2m
{

template<class T, class C>
void parallel_for_each(std::vector<T>& ts, C callable, uint poolsize = std::thread::hardware_concurrency()) {
#if defined(C2M_PARALLEL)
#if 0 //defined(__cpp_lib_parallel_algorithm)
    std::for_each(std::execution::par_unseq, ts.begin(), ts.end(), callable);
#elif 0
    std::atomic<size_t> index(0);
    std::vector<std::thread> threads;

    for (uint i = 0; i < poolsize; i++) {
        threads.emplace_back([&]() {
            while(index < ts.size()) {
                callable(ts[index++]);
            }
        });
    }
    for (auto&& thread : threads) {
        thread.join();
    }
#else
    ThreadPool pool(poolsize);
    std::vector<std::future<typename std::result_of<C(T)>>> results;
    for (auto && t: ts)
        pool.enqueue(callable, t);

    for (auto &&result: results)
        result.wait();
#endif
#else
    (void)poolsize;
    for (auto && t: ts)
        callable(t);
#endif
}

} // namespace c2m

#endif
