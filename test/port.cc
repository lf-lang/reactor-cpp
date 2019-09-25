/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "catch2/catch.hpp"

#include "dear/port.hh"

using namespace dear;

TEST_CASE("create input ports", "[port]") {
  Reactor r1("r1");
  Port<int> i1("i1", PortType::Input, &r1);
  REQUIRE(i1.name() == "i1");
  REQUIRE(i1.fqn() == "r1.i1");
  REQUIRE(i1.is_input());
  REQUIRE_FALSE(i1.is_output());
  REQUIRE_FALSE(i1.has_inward_binding());
  REQUIRE_FALSE(i1.has_outward_bindings());
  REQUIRE(i1.inward_binding() == nullptr);
  REQUIRE(i1.outward_bindings().size() == 0);
  REQUIRE(i1.typed_inward_binding() == nullptr);
  REQUIRE(i1.typed_outward_bindings().size() == 0);

  // try again in nested reactor
  Reactor r2("r2", &r1);
  Port<int> i2("i2", PortType::Input, &r2);
  REQUIRE(i2.name() == "i2");
  REQUIRE(i2.fqn() == "r1.r2.i2");
  REQUIRE(i2.is_input());
  REQUIRE_FALSE(i2.is_output());
  REQUIRE_FALSE(i2.has_inward_binding());
  REQUIRE_FALSE(i2.has_outward_bindings());
  REQUIRE(i2.inward_binding() == nullptr);
  REQUIRE(i2.outward_bindings().size() == 0);
  REQUIRE(i2.typed_inward_binding() == nullptr);
  REQUIRE(i2.typed_outward_bindings().size() == 0);
}

TEST_CASE("create output ports", "[port]") {
  Reactor r1("r1");
  Port<int> o1("o1", PortType::Output, &r1);
  REQUIRE(o1.name() == "o1");
  REQUIRE(o1.fqn() == "r1.o1");
  REQUIRE_FALSE(o1.is_input());
  REQUIRE(o1.is_output());
  REQUIRE_FALSE(o1.has_inward_binding());
  REQUIRE_FALSE(o1.has_outward_bindings());
  REQUIRE(o1.inward_binding() == nullptr);
  REQUIRE(o1.outward_bindings().size() == 0);
  REQUIRE(o1.typed_inward_binding() == nullptr);
  REQUIRE(o1.typed_outward_bindings().size() == 0);

  // try again in nested reactor
  Reactor r2("r2", &r1);
  Port<int> o2("o2", PortType::Output, &r2);
  REQUIRE(o2.name() == "o2");
  REQUIRE(o2.fqn() == "r1.r2.o2");
  REQUIRE_FALSE(o2.is_input());
  REQUIRE(o2.is_output());
  REQUIRE_FALSE(o2.has_inward_binding());
  REQUIRE_FALSE(o2.has_outward_bindings());
  REQUIRE(o2.inward_binding() == nullptr);
  REQUIRE(o2.outward_bindings().size() == 0);
  REQUIRE(o2.typed_inward_binding() == nullptr);
  REQUIRE(o2.typed_outward_bindings().size() == 0);
}

TEST_CASE("bind ports", "[port]") {
  Reactor r1("r1");
  Port<int> i1("i1", PortType::Input, &r1);
  Port<int> o1("o1", PortType::Output, &r1);
  Reactor r2("r2", &r1);
  Port<int> i2("i2", PortType::Input, &r2);
  Port<int> o2("o2", PortType::Output, &r2);
  Reactor r3("r3", &r1);
  Port<int> i3("i3", PortType::Input, &r3);
  Port<int> o3("o3", PortType::Output, &r3);
  Reactor r4("r4", &r1);
  Port<int> i4("i4", PortType::Input, &r4);

  i1.bind_to(&i2);
  o2.bind_to(&i3);
  o2.bind_to(&i4);
  o3.bind_to(&o1);

  REQUIRE(o1.has_inward_binding());
  REQUIRE(o1.inward_binding() == &o3);
  REQUIRE(i3.has_inward_binding());
  REQUIRE(i3.inward_binding() == &o2);
  REQUIRE(i4.has_inward_binding());
  REQUIRE(i4.inward_binding() == &o2);
  REQUIRE(i2.has_inward_binding());
  REQUIRE(i2.inward_binding() == &i1);

  REQUIRE(i1.has_outward_bindings());
  REQUIRE(i1.outward_bindings().size() == 1);
  REQUIRE(i1.outward_bindings().find(&i2) != i1.outward_bindings().end());
  REQUIRE(o2.has_outward_bindings());
  REQUIRE(o2.outward_bindings().size() == 2);
  REQUIRE(o2.outward_bindings().find(&i3) != o2.outward_bindings().end());
  REQUIRE(o2.outward_bindings().find(&i4) != o2.outward_bindings().end());
  REQUIRE(o3.has_outward_bindings());
  REQUIRE(o3.outward_bindings().size() == 1);
  REQUIRE(o3.outward_bindings().find(&o1) != o3.outward_bindings().end());
}
