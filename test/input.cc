/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "catch2/catch.hpp"

#include "dear/input.hh"

using namespace dear;

TEST_CASE("create inputs", "[input]") {
  Reactor r1("r1");
  Input<int> i1("i1", &r1);
  REQUIRE(i1.name() == "i1");
  REQUIRE(i1.fqn() == "r1.i1");
  REQUIRE_FALSE(i1.is_bound());
  REQUIRE(i1.get_binding() == nullptr);
  REQUIRE(i1.get_typed_binding() == nullptr);

  // try again in nested reactor
  Reactor r2("r2", &r1);
  Input<int> i2("i2", &r2);
  REQUIRE(i2.name() == "i2");
  REQUIRE(i2.fqn() == "r1.r2.i2");
  REQUIRE_FALSE(i2.is_bound());
  REQUIRE(i2.get_binding() == nullptr);
  REQUIRE(i2.get_typed_binding() == nullptr);
}
