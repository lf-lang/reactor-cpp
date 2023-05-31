/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_PORT_HH
#define REACTOR_CPP_PORT_HH

#include <set>
#include <vector>

#include "assert.hh"
#include "fwd.hh"
#include "multiport.hh"
#include "reactor.hh"
#include "value_ptr.hh"
#include "enums.hh"

namespace reactor {

template<class T>
class Connection;

enum class PortType { Input, Output, Delay };

class BasePort : public ReactorElement {
private:
  const PortType type_;
  std::size_t tag_{0};

  // triggers and dependencies
  std::set<Reaction*> dependencies_{};
  std::set<Reaction*> triggers_{};
  std::set<Reaction*> anti_dependencies_{};

  // dynamic call back to facilitate enclaved, delayed and federated connections
  PortCallback set_callback_{nullptr};
  PortCallback clean_callback_{nullptr};

protected:
  bool present_{false}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

  BasePort(const std::string& name, PortType type, Reactor* container)
      : ReactorElement(name, match_port_enum(type), container)
      , type_(type) {

    // environment()->register_port(this);
  }

  void register_dependency(Reaction* reaction, bool is_trigger) noexcept;
  void register_antidependency(Reaction* reaction) noexcept;

  [[nodiscard]] virtual auto get_upstream_reaction() const noexcept -> std::set<Reaction*> {
    return anti_dependencies_;
  };
  [[nodiscard]] virtual auto get_downstream_reactions() const noexcept -> std::set<Reaction*> { return dependencies_; };

  virtual void cleanup() = 0;

  static auto match_port_enum(PortType type) noexcept -> ReactorElement::Type {
    switch (type) {
    case PortType::Output:
      return ReactorElement::Type::Output;
    case PortType::Input:
      return ReactorElement::Type::Input;
    default:
      return ReactorElement::Type::Port;
    };
  }

  inline void invoke_set_callback() noexcept {
    if (set_callback_ != nullptr) {
      set_callback_(*this);
    }
  }

  inline void invoke_clean_callback() noexcept {
    if (clean_callback_ != nullptr) {
      clean_callback_(*this);
    }
  }

public:
  [[nodiscard]] inline auto is_input() const noexcept -> bool { return type_ == PortType::Input; }
  [[nodiscard]] inline auto is_output() const noexcept -> bool { return type_ == PortType::Output; }
  [[nodiscard]] inline auto port_type() const noexcept -> PortType { return type_; }

  [[nodiscard]] inline auto has_dependencies() const noexcept -> bool { return !dependencies_.empty(); }
  [[nodiscard]] inline auto triggers() const noexcept -> const auto& { return triggers_; }
  [[nodiscard]] inline auto dependencies() const noexcept -> const auto& { return dependencies_; }

  [[nodiscard]] virtual auto is_present() const noexcept -> bool = 0;
  [[nodiscard]] inline auto get_tag() const noexcept -> std::size_t { return tag_; };
  [[nodiscard]] virtual auto untyped_inward_binding() const noexcept -> BasePort* = 0;
  [[nodiscard]] virtual auto has_inward_binding() const noexcept -> bool = 0;
  [[nodiscard]] inline auto anti_dependencies() const noexcept -> std::set<Reaction*> { return anti_dependencies_; }

  inline void set_tag(std::size_t tag) noexcept { tag_ = tag; }

  void register_set_callback(const PortCallback& callback);
  void register_clean_callback(const PortCallback& callback);

  friend class Reaction;
  friend class Scheduler;
  virtual void set_inward_binding(BasePort* port) noexcept = 0;
  virtual void add_outward_binding(BasePort* port) noexcept = 0;
};

template <class T> class Port : public BasePort {
private:
  // graph
  Port<T>* inward_binding_{nullptr};
  std::set<Port<T>*> outward_bindings_{};

  // stored value
  ImmutableValuePtr<T> value_ptr_{nullptr};

  void cleanup() noexcept final {
    value_ptr_ = nullptr;
    present_ = false;
    invoke_clean_callback();
  }

public:
  using value_type = T;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void set_inward_binding(BasePort* port) noexcept override { inward_binding_ = static_cast<Port<T>*>(port); }

  void add_outward_binding(BasePort* port) noexcept override {
    outward_bindings_.insert(static_cast<Port<T>*>(port)); // NOLINT
  }

  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<T>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<T>*>&;

  [[nodiscard]] inline auto is_present() const noexcept -> bool override {
    if (has_inward_binding()) {
      return inward_binding_->is_present();
    }
    return present_;
  };

  [[nodiscard]] inline auto untyped_inward_binding() const noexcept -> BasePort* override { return inward_binding_; }
  [[nodiscard]] inline auto has_inward_binding() const noexcept -> bool override { return inward_binding_ != nullptr; }
  [[nodiscard]] inline auto has_outward_bindings() const noexcept -> bool { return !outward_bindings_.empty(); }

  virtual void set(const ImmutableValuePtr<T>& value_ptr);
  void set(MutableValuePtr<T>&& value_ptr) { set(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr))); }
  void set(const T& value) { set(make_immutable_value<T>(value)); }
  void set(T&& value) { set(make_immutable_value<T>(std::forward<T>(value))); }
  // Setting a port to nullptr is not permitted.
  void set(std::nullptr_t) = delete;
  void startup() final {}
  void shutdown() final {}

  auto get() const noexcept -> const ImmutableValuePtr<T>&;
};

template <> class Port<void> : public BasePort {
private:
  // graph
  Port<void>* inward_binding_{nullptr};
  std::set<Port<void>*> outward_bindings_{};

  void cleanup() noexcept final {
    present_ = false;
    invoke_clean_callback();
  }

public:
  using value_type = void;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void set_inward_binding(BasePort* port) noexcept override {
    inward_binding_ = static_cast<Port<void>*>(port); // NOLINT
  }

  void add_outward_binding(BasePort* port) noexcept override {
    outward_bindings_.insert(static_cast<Port<void>*>(port)); // NOLINT
  }

  [[nodiscard]] inline auto is_present() const noexcept -> bool override {
    if (has_inward_binding()) {
      return inward_binding_->is_present();
    }
    return present_;
  };

  [[nodiscard]] inline auto untyped_inward_binding() const noexcept -> BasePort* override { return inward_binding_; }
  [[nodiscard]] inline auto has_inward_binding() const noexcept -> bool override { return inward_binding_ != nullptr; }
  [[nodiscard]] inline auto has_outward_bindings() const noexcept -> bool { return !outward_bindings_.empty(); }

  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<void>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<void>*>&;

  auto pull_connection(ConnectionProperties* properties) -> Connection<void>*;

  void set();

  void startup() final {}
  void shutdown() final {}
};

template <class T> class Input : public Port<T> { // NOLINT
public:
  Input(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Input, container) {}

  Input(Input&&) = default; // NOLINT(performance-noexcept-move-constructor)
};

template <class T> class Output : public Port<T> { // NOLINT
public:
  Output(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Output, container) {}

  Output(Output&&) = default; // NOLINT(performance-noexcept-move-constructor)
};

} // namespace reactor

#include "impl/port_impl.hh"

#endif // REACTOR_CPP_PORT_HH
