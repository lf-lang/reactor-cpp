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

namespace reactor {

class Semaphore {
 private:
  int count;
  std::mutex mutex;

 public:
  Semaphore(int count) : count(count) {}

  void release(int i) {
    {
      std::lock_guard<std::mutex> lg(mutex);
      count += i;
    }
  }

  void acquire() {
    std::unique_lock<std::mutex> lg(mutex);
    while (count <= 0) {
      lg.unlock();
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(10us);
      lg.lock();
    }
    count--;
  }
};

}  // namespace reactor
