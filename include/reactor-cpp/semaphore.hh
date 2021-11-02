/*
 * Copyright (C) 2021 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "reactor-cpp/logging.hh"


namespace reactor {

class Semaphore {
 private:
  std::atomic<int> count;

  bool try_acquire() {
    int current = count.load(std::memory_order_relaxed);
    if (current > 0) {
      int desired = current - 1;
      return count.compare_exchange_weak(current, desired,
                                         std::memory_order_acquire,
                                         std::memory_order_relaxed);
    }
    return false;
  }

 public:
  Semaphore(int count) : count(count) {}

  void release(int i) {
    int current = count.load(std::memory_order_relaxed);
    int desired{0};
    bool success = false;
    do {
      desired = current + i;
      success = count.compare_exchange_weak(current, desired,
                                            std::memory_order_release,
                                            std::memory_order_relaxed);
    } while (!success);
  }

  void acquire() {
    size_t tries{0};
    while (!try_acquire()) {
      tries++;
      //std::this_thread::sleep_for(std::chrono::microseconds(1));
      std::this_thread::yield();
    }
    reactor::log::Warn() << tries;
  }
};

}  // namespace reactor
