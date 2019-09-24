/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "catch2/catch.hpp"

#include "dear/output.hh"

using namespace dear;

TEST_CASE("create outputs", "[output]") {
  Reactor r1("r1");
  Output<int> o1("o1", &r1);
  REQUIRE(o1.name() == "o1");
  REQUIRE(o1.fqn() == "r1.o1");
  REQUIRE_FALSE(o1.is_bound());
  REQUIRE(o1.get_bindings().size() == 0);
  REQUIRE(o1.get_typed_bindings().size() == 0);

  // try again in nested reactor
  Reactor r2("r2", &r1);
  Output<int> o2("o2", &r2);
  REQUIRE(o2.name() == "o2");
  REQUIRE(o2.fqn() == "r1.r2.o2");
  REQUIRE_FALSE(o2.is_bound());
  REQUIRE(o2.get_bindings().size() == 0);
  REQUIRE(o2.get_typed_bindings().size() == 0);
}

TEST_CASE("bind output to multiple inputs", "[output]") {
  Reactor r1("r1");
  Input<int> i1("i1", &r1);
  Input<int> i2("i2", &r1);
  Reactor r2("r2");
  Input<int> i3("i3", &r2);
  Reactor r3("r3");
  Output<int> o("o", &r3);

  REQUIRE_FALSE(o.is_bound());
  REQUIRE_FALSE(i1.is_bound());
  REQUIRE_FALSE(i2.is_bound());
  REQUIRE_FALSE(i3.is_bound());
  REQUIRE(o.get_bindings().size() == 0);
  REQUIRE(o.get_typed_bindings().size() == 0);

  o.bind(&i1);
  REQUIRE(o.is_bound());
  REQUIRE(i1.is_bound());
  auto& bindings = o.get_bindings();
  auto& typed_bindings = o.get_typed_bindings();
  REQUIRE(bindings.size() == 1);
  REQUIRE(typed_bindings.size() == 1);
  REQUIRE(bindings.find(&i1) != bindings.end());
  REQUIRE(typed_bindings.find(&i1) != typed_bindings.end());
  REQUIRE(i1.get_binding() == &o);

  o.bind(&i2);
  REQUIRE(o.is_bound());
  REQUIRE(i2.is_bound());
  REQUIRE(bindings.size() == 2);
  REQUIRE(typed_bindings.size() == 2);
  REQUIRE(bindings.find(&i2) != bindings.end());
  REQUIRE(typed_bindings.find(&i2) != typed_bindings.end());
  REQUIRE(i2.get_binding() == &o);

  o.bind(&i3);
  REQUIRE(o.is_bound());
  REQUIRE(i3.is_bound());
  REQUIRE(bindings.size() == 3);
  REQUIRE(typed_bindings.size() == 3);
  REQUIRE(bindings.find(&i3) != bindings.end());
  REQUIRE(typed_bindings.find(&i3) != typed_bindings.end());
  REQUIRE(i3.get_binding() == &o);
}
