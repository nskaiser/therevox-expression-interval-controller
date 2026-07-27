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

  assert(microvoltsToMcp4725Code(0) == 0);
  assert(microvoltsToMcp4725Code(kPicoDacFullScaleMicrovolts) == 4095);
  assert(microvoltsToMcp4725Code(1000000) >= 1240);
  assert(microvoltsToMcp4725Code(1000000) <= 1242);
  assert(microvoltsToMcp4725Code(3000000) >= 3722);
  assert(microvoltsToMcp4725Code(3000000) <= 3724);

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

  printf("pico core tests passed\n");
  return 0;
}
