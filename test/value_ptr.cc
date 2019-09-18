#include "catch2/catch.hpp"

#include "dear/value_ptr.hh"

using namespace dear;

TEST_CASE("check class properties of ValuePtr", "[value_ptr]") {
  REQUIRE(std::is_default_constructible<ValuePtr<int>>::value);
  REQUIRE_FALSE(std::is_copy_assignable<ValuePtr<int>>::value);
  REQUIRE_FALSE(std::is_copy_constructible<ValuePtr<int>>::value);
  REQUIRE(std::is_move_constructible<ValuePtr<int>>::value);
  REQUIRE(std::is_move_assignable<ValuePtr<int>>::value);
}
