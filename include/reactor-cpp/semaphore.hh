/*
 * Copyright (C) 2021 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <atomic>
#include <mutex>

namespace reactor {

class BaseSemaphore {
 private:
  int count;
  std::mutex mutex;
  std::condition_variable cv;

 public:
  BaseSemaphore(int count) : count(count) {}

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

class Semaphore {
 private:
  BaseSemaphore sem{0};
  std::atomic<int> count;

 public:
  Semaphore(int count) : count(count) {}

  void release(int i) {
    int old_count = count.fetch_add(i, std::memory_order_release);
    if (old_count < 0) {
      sem.release(std::min(i, -old_count));
    }
  }

  void acquire() {
    int old_count = count.fetch_sub(1, std::memory_order_acquire);
    if (old_count < 1) {
      sem.acquire();
    }
  }
};

}  // namespace reactor
