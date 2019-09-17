#include "catch2/catch.hpp"

#include "dear/time.hh"

using namespace dear;

TEST_CASE("time units fit", "[time]") {
  REQUIRE(1ULL == 1_ns);
  REQUIRE(1'000ULL == 1_us);
  REQUIRE(1'000'000ULL == 1_ms);
  REQUIRE(1'000'000'000ULL == 1_s);
  REQUIRE(60'000'000'000ULL == 1_min);
  REQUIRE(3'600'000'000'000ULL == 1_hr);
}

TEST_CASE("access to physical clock", "[time]") {
  auto t1 = get_physical_timepoint();
  auto t2 = get_physical_timepoint();

  REQUIRE(t1 < t2);
  assert_physical_clock_resolution();
}
