#include <assert.h>
#include <stdio.h>

#include "../pico/PrecisionExpressionControllerPico/PicoFirmwareCore.h"

using namespace expctrl;

int main() {
  assert(clampPicoSemitones(-30) == -9);
  assert(clampPicoSemitones(30) == 9);
  assert(clampPicoSemitones(7) == 7);

  assert(picoSemitonesToMicrovolts(9) == 1125000);
  assert(picoSemitonesToMicrovolts(-9) == -1125000);
  assert(picoSemitonesToMicrovolts(12) == 1125000);
  assert(picoSemitonesToMicrovolts(-12) == -1125000);
  assert(picoSemitonesToMicrovolts(7) == 875000);

  assert(computePicoCenteredOutputMicrovolts(0.0f, 9) == 1500000);
  assert(computePicoCenteredOutputMicrovolts(1.0f, 9) == 2625000);
  assert(computePicoCenteredOutputMicrovolts(0.0f, -9) == 1500000);
  assert(computePicoCenteredOutputMicrovolts(1.0f, -9) == 375000);
  assert(computePicoCenteredOutputMicrovolts(0.5f, 9) == 2062500);
  assert(computePicoCenteredOutputMicrovolts(0.5f, -9) == 937500);
  assert(computePicoCenteredOutputMicrovolts(0.75f, 0) == 1500000);
  assert(computePicoCenteredOutputMicrovolts(1.0f, 4, 1500000, 4500000) == 3000000);
  assert(computePicoCenteredOutputMicrovolts(1.0f, 12, 1500000, 8000000) ==
         kPicoDacFullScaleMicrovolts);
  assert(computePicoMappedOutputMicrovolts(0.0f, 1500000, 2750000) == 1500000);
  assert(computePicoMappedOutputMicrovolts(1.0f, 1500000, 2750000) == 2750000);
  assert(computePicoMappedOutputMicrovolts(0.5f, 1500000, 2750000) == 2125000);
  assert(computePicoMappedOutputMicrovolts(1.0f, 1500000, -500000) == 0);
  assert(picoIntervalMagnitude(-12) == 9);
  assert(picoIntervalMagnitude(3) == 3);
  assert(signedPicoInterval(7, kPicoBendUp) == 7);
  assert(signedPicoInterval(7, kPicoBendDown) == -7);
  assert(picoUnipolarNoBendMicrovolts(kPicoBendUp) == 0);
  assert(picoUnipolarNoBendMicrovolts(kPicoBendDown) == kPicoDacFullScaleMicrovolts);
  assert(computePicoUnipolarToeMicrovolts(9) == 2475000);
  assert(computePicoUnipolarToeMicrovolts(-9) == 825000);
  assert(computePicoUnipolarToeMicrovolts(12) == 2475000);
  assert(computePicoUnipolarToeMicrovolts(-12) == 825000);
  assert(computePicoUnipolarToeMicrovolts(3) == 825000);
  assert(computePicoUnipolarToeMicrovolts(-3) == 2475000);
  assert(computePicoResponseToeMicrovolts(1, 924) >= 356000);
  assert(computePicoResponseToeMicrovolts(1, 924) <= 358000);
  assert(computePicoResponseToeMicrovolts(4, 924) >= 1428000);
  assert(computePicoResponseToeMicrovolts(4, 924) <= 1430000);
  assert(computePicoResponseToeMicrovolts(9, 924) >= 3213000);
  assert(computePicoResponseToeMicrovolts(9, 924) <= 3215000);
  assert(computePicoResponseToeMicrovolts(10, 924) >= 3213000);
  assert(computePicoResponseToeMicrovolts(10, 924) <= 3215000);
  assert(computePicoResponseToeMicrovolts(-3, 924) >= 2228000);
  assert(computePicoResponseToeMicrovolts(-3, 924) <= 2230000);
  assert(computePicoResponseToeMicrovolts(9, 3960) >= 749000);
  assert(computePicoResponseToeMicrovolts(9, 3960) <= 751000);
  assert(computePicoResponseToeMicrovolts(12, 3960) >= 749000);
  assert(computePicoResponseToeMicrovolts(12, 3960) <= 751000);
  assert(computePicoResponseToeMicrovolts(4, 3960) >= 332000);
  assert(computePicoResponseToeMicrovolts(4, 3960) <= 334000);

  assert(microvoltsToMcp4728Code(0) == 0);
  assert(microvoltsToMcp4728Code(kPicoDacFullScaleMicrovolts) == 4095);
  assert(microvoltsToMcp4728Code(1000000) >= 1240);
  assert(microvoltsToMcp4728Code(1000000) <= 1242);
  assert(microvoltsToMcp4728Code(3000000) >= 3722);
  assert(microvoltsToMcp4728Code(3000000) <= 3724);

  assert(picoAbsoluteIntervalLabel(0)[0] == '1');
  assert(picoAbsoluteIntervalLabel(1)[0] == 'b');
  assert(picoAbsoluteIntervalLabel(9)[0] == '6');
  assert(picoAbsoluteIntervalLabel(10)[0] == '?');
  assert(clampPicoCurveMode(-1) == kPicoCurveLinear);
  assert(clampPicoCurveMode(99) == kPicoCurveSmooth);
  assert(picoCurveName(kPicoCurveEaseOut)[0] == 'e');
  assert(picoCurveDisplayLabel(kPicoCurveSmooth)[0] == 'S');

  PedalCalibration cal;
  cal.heelRaw = 0;
  cal.toeRaw = 1000;
  cal.snapLow = 0.0f;
  cal.snapHigh = 1.0f;
  cal.deadband = 0.0f;

  PedalFilterSettings filter;
  filter.enabled = false;

  PedalProcessor processor;
  processor.configure(cal, filter, kPicoCurveLinear);
  PedalState linear = processor.process(250, 1000.0f);
  assert(linear.curved > 0.24f && linear.curved < 0.26f);

  processor.configure(cal, filter, kPicoCurveEaseOut);
  processor.reset();
  PedalState easeOut = processor.process(250, 1000.0f);
  assert(easeOut.curved > linear.curved);

  processor.configure(cal, filter, kPicoCurveSquare);
  processor.reset();
  PedalState square = processor.process(250, 1000.0f);
  assert(square.curved < linear.curved);

  assert(picoOutputModeDisplayLabel(kPicoOutputPedal)[0] == 'P');
  assert(picoOutputModeDisplayLabel(kPicoOutputLfoLo)[0] == 'L');
  assert(picoLfoWaveDisplayLabel(kPicoLfoTriangle)[0] == 'T');
  assert(clampPicoOutputMode(99) == kPicoOutputLfoFm);
  assert(clampPicoLfoWave(99) == kPicoLfoDrift);
  assert(clampPicoLfoRateStep(999, kPicoOutputLfoLo) == kPicoLfoLoRateSteps);
  assert(clampPicoLfoRateStep(999, kPicoOutputLfoFm) == kPicoLfoFmRateSteps);
  assert(computePicoLfoRateHz(kPicoOutputLfoLo, kPicoDefaultLfoLoRate) > 0.95f);
  assert(computePicoLfoRateHz(kPicoOutputLfoLo, kPicoDefaultLfoLoRate) < 1.05f);
  assert(computePicoLfoRateHz(kPicoOutputLfoFm, kPicoDefaultLfoFmRate) > 31.0f);
  assert(computePicoLfoRateHz(kPicoOutputLfoFm, kPicoDefaultLfoFmRate) < 33.0f);
  assert(computePicoLfoRateHzForPedal(kPicoOutputLfoLo, 0.0f, kPicoLfoLoRateSteps) > 0.049f);
  assert(computePicoLfoRateHzForPedal(kPicoOutputLfoLo, 0.0f, kPicoLfoLoRateSteps) < 0.051f);
  assert(computePicoLfoRateHzForPedal(kPicoOutputLfoLo, 0.5f, kPicoLfoLoRateSteps) > 0.99f);
  assert(computePicoLfoRateHzForPedal(kPicoOutputLfoLo, 0.5f, kPicoLfoLoRateSteps) < 1.01f);
  assert(computePicoLfoRateHzForPedal(kPicoOutputLfoLo, 1.0f, kPicoLfoLoRateSteps) > 19.9f);
  assert(computePicoLfoRateHzForPedal(kPicoOutputLfoLo, 1.0f, kPicoLfoLoRateSteps) < 20.1f);
  assert(nearestPicoLfoRateStep(kPicoOutputLfoLo, 1.0f) == kPicoDefaultLfoLoRate);
  assert(clampPicoLfoDepthPercent(-5) == 0);
  assert(clampPicoLfoDepthPercent(49) == 50);
  assert(clampPicoLfoDepthPercent(73) == 75);
  assert(clampPicoLfoDepthPercent(101) == 100);
  assert(picoLfoDepthPercentFromIndex(0) == 0);
  assert(picoLfoDepthPercentFromIndex(kPicoLfoDepthStepCount - 1) == 100);
  assert(attenuatePicoLfoWaveValue(1.0f, 75) > 0.874f);
  assert(attenuatePicoLfoWaveValue(1.0f, 75) < 0.876f);
  assert(attenuatePicoLfoWaveValue(0.0f, 75) > 0.124f);
  assert(attenuatePicoLfoWaveValue(0.0f, 75) < 0.126f);
  assert(attenuatePicoLfoWaveValue(1.0f, 0) > 0.499f);
  assert(attenuatePicoLfoWaveValue(1.0f, 0) < 0.501f);
  assert(computePicoLfoWaveValue(0.25f, kPicoLfoSine) > 0.99f);
  assert(computePicoLfoWaveValue(0.25f, kPicoLfoTriangle) > 0.49f);
  assert(computePicoLfoWaveValue(0.25f, kPicoLfoSawUp) > 0.24f);
  assert(computePicoLfoWaveValue(0.25f, kPicoLfoSawDown) > 0.74f);
  assert(computePicoLfoWaveValue(0.60f, kPicoLfoSquare) < 0.01f);
  assert(computePicoLfoWaveValue(0.20f, kPicoLfoPulse) > 0.99f);
  assert(computePicoLfoWaveValue(0.30f, kPicoLfoPulse) < 0.01f);

  // Offset shifts and clamps.
  assert(clampPicoLfoOffsetPercent(-60) == -50);
  assert(clampPicoLfoOffsetPercent(23) == 25);
  assert(clampPicoLfoOffsetPercent(60) == 50);
  assert(picoLfoOffsetPercentFromIndex(0) == -50);
  assert(picoLfoOffsetPercentFromIndex(kPicoLfoOffsetStepCount - 1) == 50);
  assert(offsetPicoLfoWaveValue(0.5f, 25) > 0.749f);
  assert(offsetPicoLfoWaveValue(0.5f, 25) < 0.751f);
  assert(offsetPicoLfoWaveValue(0.9f, 25) > 0.999f);
  assert(offsetPicoLfoWaveValue(0.1f, -25) < 0.001f);

  // Pulse width.
  assert(clampPicoLfoPulseWidth(0) == 5);
  assert(clampPicoLfoPulseWidth(52) == 50);
  assert(clampPicoLfoPulseWidth(100) == 95);
  assert(computePicoLfoWaveValue(0.70f, kPicoLfoPulse, 0, 0, 75) > 0.99f);
  assert(computePicoLfoWaveValue(0.80f, kPicoLfoPulse, 0, 0, 75) < 0.01f);

  // Random waves: deterministic per cycle, in range, and actually varying.
  float sh1 = computePicoLfoWaveValue(0.1f, kPicoLfoSampleHold, 7, 42);
  float sh2 = computePicoLfoWaveValue(0.9f, kPicoLfoSampleHold, 7, 42);
  float sh3 = computePicoLfoWaveValue(0.1f, kPicoLfoSampleHold, 8, 42);
  assert(sh1 == sh2);
  assert(sh1 != sh3);
  assert(sh1 >= 0.0f && sh1 < 1.0f);
  float driftStart = computePicoLfoWaveValue(0.0f, kPicoLfoDrift, 7, 42);
  float driftEnd = computePicoLfoWaveValue(0.999f, kPicoLfoDrift, 7, 42);
  float nextStart = computePicoLfoWaveValue(0.0f, kPicoLfoDrift, 8, 42);
  assert(fabsf(driftStart - sh1) < 0.0001f);
  assert(fabsf(driftEnd - nextStart) < 0.01f);

  // Link ratios and quadrature phase.
  assert(picoLfoLinkNumerator(kPicoLfoLink1to2) == 1);
  assert(picoLfoLinkDenominator(kPicoLfoLink1to2) == 2);
  assert(picoLfoLinkNumerator(kPicoLfoLink3to2) == 3);
  float linkedPhase = 0.0f;
  uint32_t linkedCycle = 0;
  computePicoLinkedPhase(0.5f, 0, kPicoLfoLink1to1, kPicoLfoPhase0, &linkedPhase, &linkedCycle);
  assert(fabsf(linkedPhase - 0.5f) < 0.001f && linkedCycle == 0);
  computePicoLinkedPhase(0.5f, 0, kPicoLfoLink1to1, kPicoLfoPhase90, &linkedPhase, &linkedCycle);
  assert(fabsf(linkedPhase - 0.75f) < 0.001f);
  computePicoLinkedPhase(0.5f, 0, kPicoLfoLink2to1, kPicoLfoPhase0, &linkedPhase, &linkedCycle);
  assert(fabsf(linkedPhase - 0.0f) < 0.001f && linkedCycle == 1);
  computePicoLinkedPhase(0.5f, 1, kPicoLfoLink1to2, kPicoLfoPhase0, &linkedPhase, &linkedCycle);
  assert(fabsf(linkedPhase - 0.75f) < 0.001f && linkedCycle == 0);
  computePicoLinkedPhase(0.0f, 4, kPicoLfoLink1to2, kPicoLfoPhase0, &linkedPhase, &linkedCycle);
  assert(fabsf(linkedPhase - 0.0f) < 0.001f && linkedCycle == 2);

  // Clock source helpers.
  assert(clampPicoClockSource(9) == kPicoClockLfo2);
  assert(picoClockSourceName(kPicoClockOff)[0] == 'o');
  assert(picoClockSourceDisplayLabel(kPicoClockLfo1)[0] == 'L');

  printf("pico core tests passed\n");
  return 0;
}
