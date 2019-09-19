/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "catch2/catch.hpp"

#include "dear/reactor.hh"

using namespace dear;

TEST_CASE("check class properties of Reactor", "[reactor]") {
  REQUIRE_FALSE(std::is_default_constructible<Reactor>::value);
  REQUIRE_FALSE(std::is_move_constructible<Reactor>::value);
  REQUIRE_FALSE(std::is_copy_constructible<Reactor>::value);
  REQUIRE_FALSE(std::is_move_assignable<Reactor>::value);
  REQUIRE_FALSE(std::is_copy_assignable<Reactor>::value);
}

TEST_CASE("check class properties of ReactorElement", "[reactor]") {
  REQUIRE_FALSE(std::is_default_constructible<ReactorElement>::value);
  REQUIRE_FALSE(std::is_move_constructible<ReactorElement>::value);
  REQUIRE_FALSE(std::is_copy_constructible<ReactorElement>::value);
  REQUIRE_FALSE(std::is_move_assignable<ReactorElement>::value);
  REQUIRE_FALSE(std::is_copy_assignable<ReactorElement>::value);
}

TEST_CASE("", "[reactor]") {
  Reactor r1("r1");
  REQUIRE(r1.name() == "r1");
  REQUIRE(r1.fqn() == "r1");
  REQUIRE(r1.container() == nullptr);
  REQUIRE(r1.is_top_level());

  Reactor r2("r2", &r1);
  REQUIRE(r2.name() == "r2");
  REQUIRE(r2.fqn() == "r1.r2");
  REQUIRE(r2.container() == &r1);
  REQUIRE(!r2.is_top_level());

  Reactor r3("r3", &r1);
  REQUIRE(r3.name() == "r3");
  REQUIRE(r3.fqn() == "r1.r3");
  REQUIRE(r3.container() == &r1);
  REQUIRE(!r3.is_top_level());

  Reactor r4("r4", &r3);
  REQUIRE(r4.name() == "r4");
  REQUIRE(r4.fqn() == "r1.r3.r4");
  REQUIRE(r4.container() == &r3);
  REQUIRE(!r4.is_top_level());
}
