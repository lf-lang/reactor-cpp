/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_PORT_HH
#define REACTOR_CPP_PORT_HH

#include <cstdint>
#include <set>
#include <vector>

#include "assert.hh"
#include "connection_properties.hh"
#include "fwd.hh"
#include "multiport.hh"
#include "reactor_element.hh"
#include "value_ptr.hh"

namespace reactor {

template <class T> class MutationChangeMultiportSize;

enum class PortType : std::uint8_t { Input, Output, Delay };

class BasePort : public ReactorElement {
private:
  BasePort* inward_binding_{nullptr};
  std::set<BasePort*> outward_bindings_{};
  PortType type_;

  std::set<Reaction*> dependencies_{};
  std::set<Reaction*> triggers_{};
  std::set<Reaction*> anti_dependencies_{};

  PortCallback set_callback_{nullptr};
  PortCallback clean_callback_{nullptr};

protected:
  bool present_{false}; // NOLINT cppcoreguidelines-non-private-member-variables-in-classes

  BasePort(const std::string& name, PortType type, Reactor* container)
      : ReactorElement(name, match_port_enum(type), container)
      , type_(type) {}

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

  void invoke_set_callback() noexcept {
    if (set_callback_ != nullptr) {
      set_callback_(*this);
    }
  }

  void invoke_clean_callback() noexcept {
    if (clean_callback_ != nullptr) {
      clean_callback_(*this);
    }
  }

public:
  void register_dependency(Reaction* reaction, bool is_trigger) noexcept;
  void register_antidependency(Reaction* reaction) noexcept;
  void set_inward_binding(BasePort* port) noexcept {
    if (port != nullptr) {
      std::cout << port->fqn() << "(" << port << ")"
                << " --> " << this->fqn() << "(" << this << ")" << std::endl;
    }

    inward_binding_ = port;
  }
  void add_outward_binding(BasePort* port) noexcept { outward_bindings_.insert(port); }

  virtual void instantiate_connection_to(const ConnectionProperties& properties,
                                         const std::vector<BasePort*>& downstreams) = 0;

  [[nodiscard]] auto is_input() const noexcept -> bool { return type_ == PortType::Input; }
  [[nodiscard]] auto is_output() const noexcept -> bool { return type_ == PortType::Output; }
  [[nodiscard]] auto is_present() const noexcept -> bool {
    if (has_inward_binding()) {
      return inward_binding()->is_present();
    }
    return present_;
  };

  [[nodiscard]] auto has_inward_binding() const noexcept -> bool { return inward_binding_ != nullptr; }
  [[nodiscard]] auto has_outward_bindings() const noexcept -> bool { return !outward_bindings_.empty(); }
  [[nodiscard]] auto has_dependencies() const noexcept -> bool { return !dependencies_.empty(); }
  [[nodiscard]] auto has_anti_dependencies() const noexcept -> bool { return !anti_dependencies_.empty(); }

  [[nodiscard]] auto inward_binding() const noexcept -> BasePort* { return inward_binding_; }
  [[nodiscard]] auto outward_bindings() const noexcept -> const auto& { return outward_bindings_; }

  [[nodiscard]] auto triggers() const noexcept -> const auto& { return triggers_; }
  [[nodiscard]] auto dependencies() const noexcept -> const auto& { return dependencies_; }
  [[nodiscard]] auto anti_dependencies() noexcept -> auto& { return anti_dependencies_; } // TODO: make it const again
  [[nodiscard]] auto port_type() const noexcept -> PortType { return type_; }

  void register_set_callback(const PortCallback& callback);
  void register_clean_callback(const PortCallback& callback);

  friend class Reaction;
  friend class Scheduler;
};

inline auto operator==(const BasePort& a, const BasePort& b) -> bool {
  bool equal = ((const ReactorElement&)a) == ((const ReactorElement&)b);
  return equal;
}

template <class T> class Port : public BasePort {
private:
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

  void instantiate_connection_to(const ConnectionProperties& properties,
                                 const std::vector<BasePort*>& downstream) override;
  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<T>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<T>*>&;

  virtual void set(const ImmutableValuePtr<T>& value_ptr);
  void set(MutableValuePtr<T>&& value_ptr) { set(ImmutableValuePtr<T>(std::move(value_ptr))); }
  void set(const T& value) { set(make_immutable_value<T>(value)); }
  void set(T&& value) { set(make_immutable_value<T>(std::move(value))); }

  // Setting a port to nullptr is not permitted. We use enable_if to only delete
  // set() if it is actually called with nullptr. Without enable_if set(0) would
  // be ambiguous as 0 can be implicitly casted to nullptr_t.
  template <typename V, typename = std::enable_if_t<std::is_same_v<V, std::nullptr_t>>> void set(V) = delete;

  void startup() final {}
  void shutdown() final {}

  auto get() const noexcept -> const ImmutableValuePtr<T>&;
};

template <> class Port<void> : public BasePort {
private:
  void cleanup() noexcept final {
    present_ = false;
    invoke_clean_callback();
  }

public:
  using value_type = void;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void instantiate_connection_to(const ConnectionProperties& properties,
                                 const std::vector<BasePort*>& downstream) override;
  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<void>*;
  [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<void>*>&;

  void set();

  void startup() final {}
  void shutdown() final {}
};

template <class T> class Input : public Port<T> { // NOLINT(cppcoreguidelines-special-member-functions)
public:
  Input(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Input, container) {}

  Input(Input&&) = default;

  ~Input() { std::cout << "Input port gets deallocated:" << this->fqn() << std::endl; }
};

template <class T> class Output : public Port<T> { // NOLINT(cppcoreguidelines-special-member-functions)
public:
  Output(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Output, container) {}

  Output(Output&&) noexcept = default;

  ~Output() { std::cout << "Output port gets deallocated: " << this->fqn() << std::endl; }
};

} // namespace reactor

#include "impl/port_impl.hh"

#endif // REACTOR_CPP_PORT_HH
