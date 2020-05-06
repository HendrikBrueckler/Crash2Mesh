#ifndef C2M_PAR_FOR_HPP
#define C2M_PAR_FOR_HPP

#include <crash2mesh/core/types.hpp>

#include <thread>
#if defined(C2M_PARALLEL)
#include <sandbox/ThreadPool/ThreadPool.h>
#endif

namespace c2m
{

template<class T, class C>
void parallel_for_each(std::vector<T>& ts, C callable, uint poolsize = std::thread::hardware_concurrency()) {
#if defined(C2M_PARALLEL)
    ThreadPool pool(poolsize);
    std::vector<std::future<typename std::result_of<C(T)>>> results;
    for (auto && t: ts)
        pool.enqueue(callable, t);

    for (auto &&result: results)
        result.wait();
#else
    (void)poolsize;
    for (auto && t: ts)
        callable(t);
#endif
}

} // namespace c2m

#endif
