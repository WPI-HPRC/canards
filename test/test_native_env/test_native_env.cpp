// Guards the native env itself: flight logic is built without Arduino.
#include <unity.h>

void setUp() {}
void tearDown() {}

static void test_arduino_is_not_defined() {
#if defined(ARDUINO)
  TEST_FAIL_MESSAGE("the native env must not define ARDUINO");
#else
  TEST_PASS();
#endif
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_arduino_is_not_defined);
  return UNITY_END();
}
