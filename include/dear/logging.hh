/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>

namespace dear {

namespace log {

class BaseLogger {
 private:
  using Lock = std::unique_lock<std::mutex>;

  unsigned log_level;
  std::string log_prefix;
  static std::mutex mutex;
  Lock lock;

 public:
  BaseLogger(unsigned log_level, std::string log_prefix)
    : log_level(log_level), log_prefix(log_prefix), lock(mutex) {
    std::cerr << log_prefix;
  }

  template <class T>
  BaseLogger& operator<<(const T& msg) {
    std::cerr << msg;
    return *this;
  }

  ~BaseLogger() {
    std::cerr << std::endl;
    // this automatically releases the lock
  }
};

struct Debug : BaseLogger {
  Debug() : BaseLogger(0, "[DEBUG] ") {}
};

struct Info : BaseLogger {
  Info() : BaseLogger(1, "[INFO]  ") {}
};

struct Warn : BaseLogger {
  Warn() : BaseLogger(2, "[WARN]  ") {}
};

struct Error : BaseLogger {
  Error() : BaseLogger(3, "[ERROR] ") {}
};

}  // namespace log

};  // namespace dear
