/*
 * Copyright (C) 2026 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they were
 * provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit this
 * software or the related documents without Intel's prior written permission.
 *
 * This software and the related documents are provided as is, with no express or
 * implied warranties, other than those that are expressly stated in the License.
 */

/**
 * \file
 * \brief Simple thread barrier.
 */

#ifndef TOOLS_THREAD_BARRIER_H
#define TOOLS_THREAD_BARRIER_H

#include <condition_variable>
#include <mutex>

/**
 * \brief Simple thread barrier class.
 *
 * Blocks until wait() has been called N times.
 */
class thread_barrier {
public:
    explicit thread_barrier(unsigned int n) : max(n) {
        if (max == 0) {
            throw std::runtime_error("invalid barrier");
        }
    }

    void wait() {
        std::unique_lock<std::mutex> lock(mtx);
        unsigned int gen = generation;
        count++;
        if (count >= max) {
            count = 0;
            generation++;
            condvar.notify_all();
        } else {
            condvar.wait(lock, [this, gen] { return gen != generation; });
        }
    }

private:
    std::mutex mtx;
    std::condition_variable condvar;
    unsigned int count = 0;
    unsigned int generation = 0;
    unsigned int max;
};

#endif // TOOLS_THREAD_BARRIER_H
