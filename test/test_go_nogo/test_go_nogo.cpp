// GNC GO / NO-GO (src/Safety/GoNoGo.h).
#include <unity.h>

#include <cmath>

#include "Safety/GoNoGo.h"

namespace {

GoNoGo::Inputs allGood() {
  GoNoGo::Inputs in;
  in.inPreFlight = true;
  in.greetingRunning = false;
  in.imuValid = in.attitudeValid = in.speedValid = in.altitudeValid = true;
  in.imuOk = in.magOk = in.baroOk = in.sdOk = true;
  in.safetySticky = 0;
  in.canardsEnabled = true;
  in.lockedOut = false;
  in.cutoffLatched = false;
  in.powered = true;
  in.tiltRad = 2.0f * FlightConfig::kDegToRad;
  in.encodersValid = true;
  return in;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_all_good_is_go() { TEST_ASSERT_EQUAL_UINT16(0, GoNoGo::nogo(allGood())); }

void test_each_failure_sets_its_own_bit() {
  struct Case {
    void (*spoil)(GoNoGo::Inputs&);
    uint16_t bit;
  };
  const Case cases[] = {
      {[](GoNoGo::Inputs& in) { in.inPreFlight = false; }, GoNoGo::kNotPreFlight},
      {[](GoNoGo::Inputs& in) { in.greetingRunning = true; }, GoNoGo::kGreeting},
      {[](GoNoGo::Inputs& in) { in.attitudeValid = false; }, GoNoGo::kNavNotReady},
      {[](GoNoGo::Inputs& in) { in.altitudeValid = false; }, GoNoGo::kNavNotReady},
      {[](GoNoGo::Inputs& in) { in.baroOk = false; }, GoNoGo::kSensorDown},
      {[](GoNoGo::Inputs& in) { in.sdOk = false; }, GoNoGo::kSensorDown},
      {[](GoNoGo::Inputs& in) { in.safetySticky = 4; }, GoNoGo::kSafetyFault},
      {[](GoNoGo::Inputs& in) { in.canardsEnabled = false; }, GoNoGo::kCanardsNotReady},
      {[](GoNoGo::Inputs& in) { in.lockedOut = true; }, GoNoGo::kCanardsNotReady},
      {[](GoNoGo::Inputs& in) { in.cutoffLatched = true; }, GoNoGo::kCanardsNotReady},
      {[](GoNoGo::Inputs& in) { in.powered = false; }, GoNoGo::kCanardsNotReady},
      {[](GoNoGo::Inputs& in) { in.tiltRad = 20.0f * FlightConfig::kDegToRad; }, GoNoGo::kTilted},
      {[](GoNoGo::Inputs& in) { in.tiltRad = NAN; }, GoNoGo::kTilted},
      {[](GoNoGo::Inputs& in) { in.encodersValid = false; }, GoNoGo::kEncoderBad},
  };
  for (const Case& c : cases) {
    GoNoGo::Inputs in = allGood();
    c.spoil(in);
    TEST_ASSERT_EQUAL_UINT16(c.bit, GoNoGo::nogo(in));
  }
}

void test_defaults_are_no_go() { TEST_ASSERT_TRUE(GoNoGo::nogo(GoNoGo::Inputs{}) != 0); }

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_all_good_is_go);
  RUN_TEST(test_each_failure_sets_its_own_bit);
  RUN_TEST(test_defaults_are_no_go);
  return UNITY_END();
}
