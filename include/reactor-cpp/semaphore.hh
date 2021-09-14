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

namespace reactor {

class Semaphore {
 private:
  int count;
  std::mutex mutex;
  std::condition_variable cv;

 public:
  Semaphore(int count) : count(count) {}

  void release(int i) {
    {
      std::lock_guard<std::mutex> lg(mutex);
      count += i;
    }
    cv.notify_all();
  }

  void acquire() {
    std::unique_lock<std::mutex> lg(mutex);
    cv.wait(lg, [&]() { return count != 0; });
    count--;
  }
};

}  // namespace reactor
