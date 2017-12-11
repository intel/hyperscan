/*
 * Copyright (c) 2016, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BOUNDEDQUEUE_H
#define BOUNDEDQUEUE_H

#include <algorithm>
#include <cassert>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <type_traits>
#include <vector>

#include <boost/core/noncopyable.hpp>

//#define QUEUE_STATS 1

#ifdef QUEUE_STATS

#include <iostream>

class BoundedQueueStats {
public:
    size_t pop = 0; //!< Number of pop operations.
    size_t pop_block = 0; //!< Number of pop operations that had to block.
    size_t push = 0; //!< Number of push operations.
    size_t push_elements = 0; //!< Number of elements pushed.
    size_t push_block = 0; //!< Number of push operations that had to block.
    size_t refill = 0; //!< Number of refills done.
    size_t stolen_from = 0; //!< Number of times we were stolen from.

    void dump() const {
        std::cout << "pop           : " << pop << std::endl;
        std::cout << "pop_block     : " << pop_block << std::endl;
        std::cout << "push          : " << push << std::endl;
        std::cout << "push_elements : " << push_elements << std::endl;
        std::cout << "push_block    : " << push_block << std::endl;
        std::cout << "refill        : " << refill << std::endl;
        std::cout << "stolen_from   : " << stolen_from << std::endl;
    }
};
#endif

template<typename T>
class BoundedQueue : boost::noncopyable {
private:
    // Encapsulates a queue and the mutex used to protect access to it.
    class MutexQueue {
    public:
        // Forwarded queue operations.
        void push(std::unique_ptr<T> elem) { q.push(std::move(elem)); }
        void pop() { q.pop(); }
        std::unique_ptr<T> &front() { return q.front(); }
        bool empty() const { return q.empty(); }
        size_t size() const { return q.size(); }

        // Acquire the mutex lock.
        std::unique_lock<std::mutex> lock() {
            return std::unique_lock<std::mutex>(mutex);
        }

#ifdef QUEUE_STATS
        BoundedQueueStats stats;
#endif

    private:
        std::mutex mutex;
        std::queue<std::unique_ptr<T>> q;
    };

public:
    BoundedQueue(size_t consumers, size_t size)
        : max_elements(size), consumer_q(consumers) {
        assert(consumers > 0);
        assert(size > 0);
    }

#ifdef QUEUE_STATS
    ~BoundedQueue() {
        std::cout << "Global queue stats:" << std::endl;
        global_q.stats.dump();
        std::cout << std::endl;
        for (size_t i = 0; i < consumer_q.size(); i++) {
            std::cout << "Consumer queue " << i << ":" << std::endl;
            consumer_q[i].stats.dump();
            std::cout << std::endl;
        }
    }
#endif // QUEUE_STATS

    void push(std::unique_ptr<T> elem) {
        auto lock = global_q.lock();

#ifdef QUEUE_STATS
        global_q.stats.push++;
        global_q.stats.push_elements++;
        if (global_q.size() >= max_elements) {
            global_q.stats.push_block++;
        }
#endif // QUEUE_STATS

        // Block until queue is able to accept new elements.
        cond_can_accept.wait(lock,
                             [&] { return global_q.size() < max_elements; });
        assert(global_q.size() < max_elements);

        global_q.push(std::move(elem));
        cond_can_consume.notify_all();
    }

    template<class Iter>
    void push(Iter begin, Iter end) {
        using ElemType = typename std::remove_reference<decltype(*begin)>::type;
        static_assert(std::is_same<ElemType, std::unique_ptr<T>>::value,
                      "Iterator must be over unique_ptr<T>");

        if (begin == end) {
            return;
        }

        auto lock = global_q.lock();

#ifdef QUEUE_STATS
        global_q.stats.push++;
        global_q.stats.push_elements += std::distance(begin, end);
        if (global_q.size() >= max_elements) {
            global_q.stats.push_block++;
        }
#endif // QUEUE_STATS

        // Block until queue is able to accept new elements.
        cond_can_accept.wait(lock,
                             [&] { return global_q.size() < max_elements; });
        assert(global_q.size() < max_elements);

        for (auto it = begin; it != end; ++it) {
            global_q.push(std::move(*it));
        }
        cond_can_consume.notify_all();
    }

    std::unique_ptr<T> pop(size_t consumer_id) {
        assert(consumer_id < consumer_q.size());
        auto &q = consumer_q[consumer_id];

        // Try and satisfy the request from our per-consumer queue.
        {
            auto consumer_lock = q.lock();
            if (!q.empty()) {
                return pop_from_queue(q);
            }
        }

        // Try and satisfy the request with a refill from the global queue.
        {
            auto lock = global_q.lock();
            if (!global_q.empty()) {
                auto consumer_lock = q.lock();
                return refill_and_pop(q);
            }
        }

        // Try and satisfy the request by stealing it from another queue.
        for (size_t i = 1; i < consumer_q.size(); i++) {
            size_t victim_id = (consumer_id + i) % consumer_q.size();
            auto &victim_q = consumer_q[victim_id];
            auto victim_lock = victim_q.lock();
            // Note: we don't steal sentinel elements.
            if (!victim_q.empty() && victim_q.front() != nullptr) {
#ifdef QUEUE_STATS
                victim_q.stats.stolen_from++;
#endif
                return pop_from_queue(victim_q);
            }
        }

        // All avenues exhausted, we must block until we've received a new
        // element.
        auto lock = global_q.lock();
#ifdef QUEUE_STATS
        global_q.stats.pop_block++;
#endif
        cond_can_consume.wait(lock, [&]{ return !global_q.empty(); });
        assert(!global_q.empty());
        auto consumer_lock = q.lock();
        return refill_and_pop(q);
    }

private:
    std::unique_ptr<T> pop_from_queue(MutexQueue &q) {
        assert(!q.empty());
        auto elem = std::move(q.front());
        q.pop();
#ifdef QUEUE_STATS
        q.stats.pop++;
#endif
        return elem;
    }

    std::unique_ptr<T> refill_and_pop(MutexQueue &q) {
        assert(!global_q.empty());

#ifdef QUEUE_STATS
        q.stats.refill++;
#endif

        auto elem = pop_from_queue(global_q);
        if (elem == nullptr) {
            return elem; // Sentinel.
        }

        // Grab all subsequent elements that share the same ID.
        const auto &id = elem->id;
        while (!global_q.empty()) {
            auto &first = global_q.front();
            if (first == nullptr) {
#ifdef QUEUE_STATS
                q.stats.push++;
                q.stats.push_elements++;
#endif
                // Sentinel element. We can grab one, but no more.
                q.push(pop_from_queue(global_q));
                break;
            }
            if (first->id != id) {
                break;
            }
#ifdef QUEUE_STATS
            q.stats.push++;
            q.stats.push_elements++;
#endif
            q.push(pop_from_queue(global_q));
        }

        if (global_q.size() < max_elements) {
            cond_can_accept.notify_all();
        }

        return elem;
    }

    // Maximum number of elements in the global queue (subsequent push
    // operations will block). Note that we may overshoot this value when
    // handling bulk push operations.
    const size_t max_elements;

    // Global queue.
    MutexQueue global_q;

    // Per-consumer queues.
    std::vector<MutexQueue> consumer_q;

    // Condition variable for producers to wait on when the queue is full.
    std::condition_variable cond_can_accept;

    // Condition variable for consumers to wait on when the queue is empty.
    std::condition_variable cond_can_consume;
};

#ifdef QUEUE_STATS
#undef QUEUE_STATS
#endif

#endif // BOUNDEDQUEUE_H
