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

#include "config.hh"

namespace reactor {

namespace log {

template <bool enabled>
class BaseLogger {};

template <>
class BaseLogger<true> {
 private:
  using Lock = std::unique_lock<std::mutex>;

  const std::string log_prefix;
  inline static std::mutex mutex{};
  Lock lock;

 public:
  BaseLogger(const std::string& log_prefix)
      : log_prefix(log_prefix), lock(mutex) {
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

template <>
class BaseLogger<false> {
 public:
  BaseLogger(const std::string&) {}

  template <class T>
  const BaseLogger& operator<<(const T&) const {
    return *this;
  }

  ~BaseLogger() {}
};

constexpr bool debug_enabled = 4 <= REACTOR_CPP_LOG_LEVEL;
constexpr bool info_enabled = 3 <= REACTOR_CPP_LOG_LEVEL;
constexpr bool warning_enabled = 2 <= REACTOR_CPP_LOG_LEVEL;
constexpr bool error_enabled = 1 <= REACTOR_CPP_LOG_LEVEL;

struct Debug : BaseLogger<debug_enabled> {
  Debug() : BaseLogger<debug_enabled>("[DEBUG] ") {}
};

struct Info : BaseLogger<info_enabled> {
  Info() : BaseLogger<info_enabled>("[INFO]  ") {}
};

struct Warn : BaseLogger<warning_enabled> {
  Warn() : BaseLogger<warning_enabled>("[WARN]  ") {}
};

struct Error : BaseLogger<error_enabled> {
  Error() : BaseLogger<error_enabled>("[ERROR] ") {}
};

}  // namespace log
}  // namespace reactor
