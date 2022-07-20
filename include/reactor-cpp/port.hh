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

#include "reactor.hh"
#include "value_ptr.hh"
#include "multiport_callback.hh"

namespace reactor {

enum class PortType { Input, Output };

class BasePort : public ReactorElement {
 private:
  BasePort* inward_binding_ = nullptr;
  std::set<BasePort*> outward_bindings_;
  const PortType type;

  std::set<Reaction*> dependencies_;
  std::set<Reaction*> triggers_;
  std::set<Reaction*> antidependencies_;

 protected:
  BasePort(const std::string& name, PortType type, Reactor* container)
      : ReactorElement(name,
                       type == PortType::Input ? ReactorElement::Type::Input
                                               : ReactorElement::Type::Output,
                       container)
      , type(type) {}

  void base_bind_to(BasePort* port);
  void register_dependency(Reaction* reaction, bool is_trigger);
  void register_antidependency(Reaction* reaction);

  virtual void cleanup() = 0;

 public:
  [[nodiscard]] auto is_input() const -> bool { return type == PortType::Input; }
  [[nodiscard]] auto is_output() const -> bool { return type == PortType::Output; }

  [[nodiscard]] auto has_inward_binding() const -> bool { return inward_binding_ != nullptr; }
  [[nodiscard]] auto has_outward_bindings() const -> bool { return !outward_bindings_.empty(); }
  [[nodiscard]] auto has_dependencies() const -> bool { return !dependencies_.empty(); }
  [[nodiscard]] auto has_anti_dependencies() const -> bool { return !antidependencies_.empty(); }

  [[nodiscard]] auto inward_binding() const -> BasePort* { return inward_binding_; }
  [[nodiscard]] auto outward_bindings() const -> const auto& { return outward_bindings_; }

  [[nodiscard]] auto triggers() const -> const auto& { return triggers_; }
  [[nodiscard]] auto dependencies() const -> const auto& { return dependencies_; }
  [[nodiscard]] auto anti_dependencies() const -> const auto& { return antidependencies_; }

  friend class Reaction;
  friend class Scheduler;
};

template <class T>
class Port : public BasePort {
 private:
  ImmutableValuePtr<T> value_ptr_{nullptr};

  void cleanup() final { value_ptr_ = nullptr; }

 public:
  using value_type = T;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<T>* port) { base_bind_to(port); }
  auto typed_inward_binding() const noexcept -> Port<T>*;
  [[maybe_unused]] auto typed_outward_bindings() const noexcept -> const std::set<Port<T>*>&;

  virtual void set(const ImmutableValuePtr<T>& value_ptr);
  virtual void set(MutableValuePtr<T>&& value_ptr) {
    set(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr)));
  }
  virtual void set(const T& value) { set(make_immutable_value<T>(value)); }
  virtual void set(T&& value) { set(make_immutable_value<T>(std::forward<T>(value))); }
  // Setting a port to nullptr is not permitted.
  virtual void set(std::nullptr_t) = delete;

  void startup() final {}
  void shutdown() final {}

  auto get() const noexcept -> const ImmutableValuePtr<T>&;
  [[nodiscard]] auto is_present() const noexcept -> bool;
};

template <>
class Port<void> : public BasePort {
 private:
  bool present_{false};

  void cleanup() final { present_ = false; }

 public:
  using value_type = void;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<void>* port) { base_bind_to(port); }
  [[nodiscard]] auto typed_inward_binding() const noexcept -> Port<void>*;
  [[maybe_unused]] [[nodiscard]] auto typed_outward_bindings() const noexcept -> const std::set<Port<void>*>&;

  void set();
  [[nodiscard]] auto is_present() const noexcept -> bool;

  void startup() final {}
  void shutdown() final {}
};

template <class T>
class Input : public Port<T> {
 private:
     callback::PortBankCallBack<Input<T>>* port_bank_ = nullptr;
 public:
  Input(const std::string& name, Reactor* container, callback::PortBankCallBack<Input<T>>* port_bank)
      : Port<T>(name, PortType::Input, container), port_bank_(port_bank) {};

  void set(const ImmutableValuePtr<T>& value_ptr){
    port_bank_->port_call_back(this);
    this->Port<T>::set(value_ptr);
  };
  void set(MutableValuePtr<T>&& value_ptr) {
    port_bank_->port_call_back(this);
    this->Port<T>::set(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr)));
  }
  void set(const T& value) { 
    port_bank_->port_call_back(this);
    this->Port<T>::set(make_immutable_value<T>(value));
  }
  void set(T&& value) {
    port_bank_->port_call_back(this);
    this->Port<T>::set(make_immutable_value<T>(std::forward<T>(value)));
  }
  // Setting a port to nullptr is not permitted.
  void set(std::nullptr_t) = delete;

  Input(Input&&)  noexcept = default;
};

template <class T>
class Output : public Port<T> {
    private:
        callback::PortBankCallBack<Output<T>>* port_bank_ = nullptr;
    public:
  Output(const std::string& name, Reactor* container, callback::PortBankCallBack<Output<T>>* port_bank)
      : Port<T>(name, PortType::Output, container), port_bank_(port_bank) {}

  Output(Output&&)  noexcept = default;

 void set(const ImmutableValuePtr<T>& value_ptr){
    port_bank_->port_call_back(this);
    this->Port<T>::set(value_ptr);
  };
  void set(MutableValuePtr<T>&& value_ptr) {
    port_bank_->port_call_back(this);
    this->Port<T>::set(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr)));
  }
  void set(const T& value) { 
    port_bank_->port_call_back(this);
    this->Port<T>::set(make_immutable_value<T>(value));
  }
  void set(T&& value) {
    port_bank_->port_call_back(this);
    this->Port<T>::set(make_immutable_value<T>(std::forward<T>(value)));
  }
  // Setting a port to nullptr is not permitted.
  void set(std::nullptr_t) = delete;


};

}  // namespace reactor

#include "impl/port_impl.hh"

#endif // REACTOR_CPP_PORT_HH