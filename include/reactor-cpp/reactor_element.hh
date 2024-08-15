/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 *   Christian Menard
 */

#ifndef REACTOR_CPP_REACTOR_ELEMENT_HH
#define REACTOR_CPP_REACTOR_ELEMENT_HH

#include <cstdint>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include "fwd.hh"

namespace reactor {
class ReactorElement { // NOLINT(cppcoreguidelines-special-member-functions)
private:
  std::string name_{};
  std::string fqn_{};

  // The reactor owning this element
  Reactor* container_{nullptr};
  Environment* environment_{};

  auto fqn_detail(std::stringstream& string_stream) const noexcept -> std::stringstream&;

public:
  enum class Type : std::uint8_t { Action, Port, Reaction, Reactor, Input, Output };

  ReactorElement(const std::string& name, Type type, Reactor* container);
  ReactorElement(const std::string& name, Type type, Environment* environment);
  virtual ~ReactorElement() = default;

  // not copyable, but movable
  ReactorElement(const ReactorElement&) = delete;
  ReactorElement(ReactorElement&&) = default;

  [[nodiscard]] auto container() const noexcept -> Reactor* { return container_; }

  [[nodiscard]] auto name() const noexcept -> const std::string& { return name_; }
  [[nodiscard]] auto fqn() const noexcept -> const std::string& { return fqn_; }
  [[nodiscard]] auto environment() noexcept -> Environment* { return environment_; }
  [[nodiscard]] auto environment() const noexcept -> const Environment* { return environment_; }

  [[nodiscard]] auto is_top_level() const noexcept -> bool { return this->container() == nullptr; }

  virtual void startup() = 0;
  virtual void shutdown() = 0;
};
} // namespace reactor

#endif // REACTOR_CPP_REACTOR_ELEMENT_HH
