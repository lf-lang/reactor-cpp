#include "catch2/catch.hpp"

#include "dear/tag.hh"

#include <type_traits>

using namespace dear;

TEST_CASE("check class properties of Tag", "[tag]") {
  REQUIRE_FALSE(std::is_default_constructible<Tag>::value);
  REQUIRE_FALSE(std::is_copy_assignable<Tag>::value);
  REQUIRE(std::is_move_constructible<Tag>::value);
  REQUIRE(std::is_copy_constructible<Tag>::value);
}

SCENARIO("Tags can be created from physical time", "[tag]") {
  dear::time_t t1 = get_physical_timepoint();
  Tag tag = Tag::from_physical_time();
  dear::time_t t2 = get_physical_timepoint();

  REQUIRE(tag.micro_step() == 0);
  REQUIRE(t1 < tag.time());
  REQUIRE(tag.time() < t2);
}

SCENARIO("Tags can be delayed", "[tag]") {
  GIVEN("a tag") {
    Tag t1 = Tag::from_physical_time();
    REQUIRE(t1.micro_step() == 0);
    REQUIRE(t1.time() > 0);

    WHEN("delayed by 0") {
      Tag t2 = t1.delay();
      THEN("the time is constant and the microstep increases") {
        REQUIRE(t2.time() == t1.time());
        REQUIRE(t2.micro_step() == t1.micro_step() + 1ull);
      }
    }

    WHEN("delayed by 1 ns") {
      Tag t2 = t1.delay(1_ns);
      THEN("the time increases by one and microstep stays 0") {
        REQUIRE(t2.time() == t1.time() + 1ull);
        REQUIRE(t2.micro_step() == 0);
      }
    }

    WHEN("delayed by 1 ns and then by 0") {
      Tag t2 = t1.delay(1_ns);
      Tag t3 = t2.delay();
      THEN("the time increases by one and microstep increases by 1") {
        REQUIRE(t3.time() == t1.time() + 1ull);
        REQUIRE(t3.micro_step() == 1);
      }
    }
    WHEN("delayed by 0 and then by 1 ns") {
      Tag t2 = t1.delay();
      Tag t3 = t2.delay(1_ns);
      THEN("the time increases by one and microstep goes back to 0") {
        REQUIRE(t3.time() == t1.time() + 1ull);
        REQUIRE(t3.micro_step() == 0);
      }
    }
  }
}

SCENARIO("Tags can be compared", "[tag]") {
  GIVEN("a tag t1") {
    Tag t1 = Tag::from_physical_time();
    REQUIRE(t1.micro_step() == 0);
    REQUIRE(t1.time() > 0);

    WHEN("copied to t2") {
      Tag t2 = t1;
      THEN("t1 == t2") {
        REQUIRE(t1 == t2);
        REQUIRE_FALSE(t1 != t2);
        REQUIRE_FALSE(t1 < t2);
        REQUIRE_FALSE(t1 > t2);
        REQUIRE(t1 <= t2);
        REQUIRE(t1 >= t2);
        REQUIRE_FALSE(t2 < t1);
        REQUIRE_FALSE(t2 > t1);
        REQUIRE(t2 <= t1);
        REQUIRE(t2 >= t1);
      }
    }

    WHEN("t2 is t1 delayed by one microstep") {
      Tag t2 = t1.delay();
      THEN("t1 < t2") {
        REQUIRE_FALSE(t1 == t2);
        REQUIRE(t1 != t2);
        REQUIRE(t1 < t2);
        REQUIRE_FALSE(t1 > t2);
        REQUIRE(t1 <= t2);
        REQUIRE_FALSE(t1 >= t2);
        REQUIRE_FALSE(t2 < t1);
        REQUIRE(t2 > t1);
        REQUIRE_FALSE(t2 <= t1);
        REQUIRE(t2 >= t1);
      }
    }

    WHEN("t2 is t1 delayed by 1 ns") {
      Tag t2 = t1.delay(1_ns);
      THEN("t1 < t2") {
        REQUIRE_FALSE(t1 == t2);
        REQUIRE(t1 != t2);
        REQUIRE(t1 < t2);
        REQUIRE_FALSE(t1 > t2);
        REQUIRE(t1 <= t2);
        REQUIRE_FALSE(t1 >= t2);
        REQUIRE_FALSE(t2 < t1);
        REQUIRE(t2 > t1);
        REQUIRE_FALSE(t2 <= t1);
        REQUIRE(t2 >= t1);
      }
    }

    WHEN("t2 is t1 delayed by 1 microstep and t3 is t1 delayed by 1ns") {
      Tag t2 = t1.delay();
      Tag t3 = t1.delay(1_ns);
      THEN("t2 < t3") {
        REQUIRE_FALSE(t2 == t3);
        REQUIRE(t2 != t3);
        REQUIRE(t2 < t3);
        REQUIRE_FALSE(t2 > t3);
        REQUIRE(t2 <= t3);
        REQUIRE_FALSE(t2 >= t3);
        REQUIRE_FALSE(t3 < t2);
        REQUIRE(t3 > t2);
        REQUIRE_FALSE(t3 <= t2);
        REQUIRE(t3 >= t2);
      }
    }
  }
}
