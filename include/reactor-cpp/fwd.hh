/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_FWD_HH
#define REACTOR_CPP_FWD_HH

namespace reactor {

class BaseAction;
class BasePort;
class Environment;
class Reaction;
class Reactor;
class Tag;

class DefaultSchedulingPolicy;
template <class SchedulingPolicy> class Worker;
template <class SchedulingPolicy> class Scheduler;

template <class T> class Action;
template <class T> class Port;

} // namespace reactor

#endif // REACTOR_CPP_FWD_HH
