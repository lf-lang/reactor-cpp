/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_REACTOR_HH
#define REACTOR_CPP_REACTOR_HH

#include <memory>
#include <set>
#include <sstream>
#include <string>

//#include "fwd.hh"
//#include "action.hh"
#include "environment.hh"
#include "logical_time.hh"
#include "reactor.hh"
#include "reactor_element.hh"

namespace reactor {

class Reactor : public ReactorElement { // NOLINT
private:
  std::set<BaseAction*> actions_{};
  std::set<BasePort*> inputs_{};
  std::set<BasePort*> outputs_{};
  std::set<Reaction*> reactions_{};
  std::set<Reactor*> reactors_{};
  std::set<BaseAction*> connections_{};

public:
  void register_action(BaseAction* action);
  void register_input(BasePort* port);
  void register_output(BasePort* port);
  void register_reaction(Reaction* reaction);
  void register_reactor(Reactor* reactor);
  void register_connection(BaseAction* connection);

  Reactor(const std::string& name, Reactor* container);
  Reactor(const std::string& name, Environment* environment);
  ~Reactor() override;

  [[nodiscard]] auto inline actions() const noexcept -> const auto& { return actions_; }
  [[nodiscard]] auto inline inputs() const noexcept -> const auto& { return inputs_; }
  [[nodiscard]] auto inline outputs() const noexcept -> const auto& { return outputs_; }
  [[nodiscard]] auto inline reactions() const noexcept -> const auto& { return reactions_; }
  [[nodiscard]] auto inline reactors() const noexcept -> const auto& { return reactors_; }

  void startup() final;
  void shutdown() final;

  virtual void assemble() = 0;

  [[nodiscard]] static auto get_physical_time() noexcept -> TimePoint;
  [[nodiscard]] auto get_logical_time() const noexcept -> TimePoint;
  [[nodiscard]] auto get_microstep() const noexcept -> mstep_t;
  [[nodiscard]] auto get_tag() const noexcept -> Tag;
  [[nodiscard]] auto get_elapsed_logical_time() const noexcept -> Duration;
  [[nodiscard]] auto get_elapsed_physical_time() const noexcept -> Duration;

  friend ReactorElement;
};

} // namespace reactor

#endif // REACTOR_CPP_REACTOR_HH
