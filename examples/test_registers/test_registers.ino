#include <unity.h>
#include <TC78B009FTG_asukiaaa/Registers.hpp>

void test_register_08() {
  TC78B009FTG_asukiaaa::Registers::R08to09 r08;
  uint8_t data[2];

  data[0] = 0b11011111;
  data[1] = 0b100;
  r08.parse(data, 2);
  TEST_ASSERT_EQUAL(0b11011111000001, r08.speedSlop);
  TEST_ASSERT_EQUAL(0, r08.maxOpen);
  TEST_ASSERT_EQUAL(0, r08.maxOff);

  data[0] = 1;
  data[1] = 1;
  r08.toBytes(data, 2);
  TEST_ASSERT_EQUAL(0b11011111, data[0]);
  TEST_ASSERT_EQUAL(0b100, data[1]);

  data[0] = 0;
  data[1] = 0b101;
  r08.parse(data, 2);
  TEST_ASSERT_EQUAL(1, r08.speedSlop);
  TEST_ASSERT_EQUAL(0, r08.maxOpen);
  TEST_ASSERT_EQUAL(1, r08.maxOff);

  data[0] = 1;
  data[1] = 0;
  r08.toBytes(data, 2);
  TEST_ASSERT_EQUAL(0, data[0]);
  TEST_ASSERT_EQUAL(0b101, data[1]);

  data[0] = 0;
  data[1] = 0b10;
  r08.parse(data, 2);
  TEST_ASSERT_EQUAL(0, r08.speedSlop);
  TEST_ASSERT_EQUAL(1, r08.maxOpen);
  TEST_ASSERT_EQUAL(0, r08.maxOff);

  data[0] = 1;
  data[1] = 0;
  r08.toBytes(data, 2);
  TEST_ASSERT_EQUAL(0, data[0]);
  TEST_ASSERT_EQUAL(0b10, data[1]);
}

void setup() {
  delay(2000);  // for board that does not support software reset
  UNITY_BEGIN();
}

void loop() {
  static int testCount = 0;
  RUN_TEST(test_register_08);
  if (++testCount >= 3) {
    UNITY_END();
  }
}
