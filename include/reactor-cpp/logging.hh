/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_LOGGING_HH
#define REACTOR_CPP_LOGGING_HH

#include "reactor-cpp/config.hh" //NOLINT
#include "reactor-cpp/time.hh"
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

namespace reactor::log {

template <bool enabled> class BaseLogger {};

template <> class BaseLogger<true> { // NOLINT
private:
  using Lock = std::unique_lock<std::mutex>;

  const std::string log_prefix_{};
  inline static std::mutex mutex_{}; // NOLINT
  Lock lock_{};

public:
  explicit BaseLogger(const std::string& log_prefix)
      : log_prefix_(log_prefix)
      , lock_(mutex_) {
    std::cerr << log_prefix;
  }

  template <class T> auto operator<<(const T& msg) -> BaseLogger& {
    std::cerr << msg; // NOLINT
    return *this;
  }

  ~BaseLogger() { std::cerr << std::endl; }
};

template <> class BaseLogger<false> { // NOLINT
public:
  explicit BaseLogger([[maybe_unused]] const std::string& /*unused*/) {}

  template <class T> auto operator<<(const T& /*unused*/) const -> const BaseLogger& { return *this; }

  ~BaseLogger() = default;
};

constexpr bool debug_enabled = 4 <= REACTOR_CPP_LOG_LEVEL;
constexpr bool info_enabled = 3 <= REACTOR_CPP_LOG_LEVEL;
constexpr bool warning_enabled = 2 <= REACTOR_CPP_LOG_LEVEL;
constexpr bool error_enabled = 1 <= REACTOR_CPP_LOG_LEVEL;

struct Debug : BaseLogger<debug_enabled> {
  Debug()
      : BaseLogger<debug_enabled>("[DEBUG] "){}; // NOLINT Update C++20
};
struct Info : BaseLogger<info_enabled> {
  Info()
      : BaseLogger<info_enabled>("[INFO]  "){}; // NOLINT Update C++20
};
struct Warn : BaseLogger<warning_enabled> {
  Warn()
      : BaseLogger<warning_enabled>("[WARN]  "){}; // NOLINT Update C++20
};
struct Error : BaseLogger<error_enabled> {
  Error()
      : BaseLogger<error_enabled>("[ERROR] "){}; // NOLINT Update C++20
};
} // namespace reactor::log

#endif // REACTOR_CPP_LOGGING_HH
