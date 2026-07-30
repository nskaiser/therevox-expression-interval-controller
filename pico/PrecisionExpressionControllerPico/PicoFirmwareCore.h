#pragma once

#include <math.h>
#include <stdint.h>

namespace expctrl {

template <typename T>
constexpr T clampValue(T value, T low, T high) {
  return value < low ? low : (value > high ? high : value);
}

constexpr int8_t kPicoMinSemitones = -9;
constexpr int8_t kPicoMaxSemitones = 9;
constexpr uint8_t kPicoSemitoneMapCount =
    static_cast<uint8_t>(kPicoMaxSemitones - kPicoMinSemitones + 1);
constexpr int32_t kPicoDacFullScaleMicrovolts = 3300000L;
constexpr uint16_t kMcp4728MaxCode = 4095;
constexpr int32_t kDefaultCenterMicrovolts = 1500000L;
constexpr int32_t kDefaultOctaveMicrovolts = 1500000L;
constexpr int32_t kDefaultUnipolarOctaveMicrovolts = kPicoDacFullScaleMicrovolts;
constexpr uint16_t kDefaultFullScaleResponseCents = 924;
constexpr uint8_t kPicoLfoLoRateSteps = 96;
constexpr uint8_t kPicoLfoFmRateSteps = 60;
constexpr uint8_t kPicoDefaultLfoLoRate = 48;
constexpr uint8_t kPicoDefaultLfoFmRate = 28;
constexpr float kPicoLfoLoMinHz = 0.05f;
constexpr float kPicoLfoLoMaxHz = 20.0f;
constexpr float kPicoLfoFmMinHz = 8.0f;
constexpr float kPicoLfoFmMaxHz = 160.0f;
constexpr uint8_t kPicoLfoMinDepthPercent = 0;
constexpr uint8_t kPicoLfoMaxDepthPercent = 100;
constexpr uint8_t kPicoLfoDepthStepPercent = 5;
constexpr uint8_t kPicoLfoDepthStepCount =
    ((kPicoLfoMaxDepthPercent - kPicoLfoMinDepthPercent) / kPicoLfoDepthStepPercent) + 1;
constexpr int8_t kPicoLfoMinOffsetPercent = -50;
constexpr int8_t kPicoLfoMaxOffsetPercent = 50;
constexpr uint8_t kPicoLfoOffsetStepPercent = 5;
constexpr uint8_t kPicoLfoOffsetStepCount =
    ((kPicoLfoMaxOffsetPercent - kPicoLfoMinOffsetPercent) / kPicoLfoOffsetStepPercent) + 1;
constexpr uint8_t kPicoLfoMinPulseWidthPercent = 5;
constexpr uint8_t kPicoLfoMaxPulseWidthPercent = 95;
constexpr uint8_t kPicoLfoPulseWidthStepPercent = 5;
constexpr uint8_t kPicoLfoPulseWidthStepCount =
    ((kPicoLfoMaxPulseWidthPercent - kPicoLfoMinPulseWidthPercent) /
     kPicoLfoPulseWidthStepPercent) + 1;
constexpr uint8_t kPicoDefaultLfoPulseWidthPercent = 25;

enum PicoBendDirection : uint8_t {
  kPicoBendUp = 0,
  kPicoBendDown = 1,
};

enum PicoOutputMode : uint8_t {
  kPicoOutputPedal = 0,
  kPicoOutputLfoLo = 1,
  kPicoOutputLfoFm = 2,
  kPicoOutputModeCount = 3,
};

enum PicoLfoWave : uint8_t {
  kPicoLfoSine = 0,
  kPicoLfoTriangle = 1,
  kPicoLfoSawUp = 2,
  kPicoLfoSawDown = 3,
  kPicoLfoSquare = 4,
  kPicoLfoPulse = 5,
  kPicoLfoSampleHold = 6,
  kPicoLfoDrift = 7,
  kPicoLfoWaveCount = 8,
};

enum PicoLfoLink : uint8_t {
  kPicoLfoLinkOff = 0,
  kPicoLfoLink1to1 = 1,
  kPicoLfoLink1to2 = 2,
  kPicoLfoLink1to4 = 3,
  kPicoLfoLink3to2 = 4,
  kPicoLfoLink2to1 = 5,
  kPicoLfoLink4to1 = 6,
  kPicoLfoLinkCount = 7,
};

enum PicoLfoPhaseOffset : uint8_t {
  kPicoLfoPhase0 = 0,
  kPicoLfoPhase90 = 1,
  kPicoLfoPhase180 = 2,
  kPicoLfoPhase270 = 3,
  kPicoLfoPhaseOffsetCount = 4,
};

enum PicoClockSource : uint8_t {
  kPicoClockOff = 0,
  kPicoClockLfo1 = 1,
  kPicoClockLfo2 = 2,
  kPicoClockSourceCount = 3,
};

enum PicoPedalCurve : uint8_t {
  kPicoCurveLinear = 0,
  kPicoCurveEaseOut = 1,
  kPicoCurveSquare = 2,
  kPicoCurveSmooth = 3,
  kPicoCurveCount = 4,
};

inline uint8_t clampPicoCurveMode(int value) {
  return static_cast<uint8_t>(clampValue<int>(value, 0, kPicoCurveCount - 1));
}

inline uint8_t clampPicoOutputMode(int value) {
  return static_cast<uint8_t>(clampValue<int>(value, 0, kPicoOutputModeCount - 1));
}

inline uint8_t clampPicoLfoOutputMode(int value) {
  return clampPicoOutputMode(value) == kPicoOutputLfoFm ? kPicoOutputLfoFm
                                                       : kPicoOutputLfoLo;
}

inline uint8_t clampPicoLfoWave(int value) {
  return static_cast<uint8_t>(clampValue<int>(value, 0, kPicoLfoWaveCount - 1));
}

inline uint8_t picoLfoRateMaxStep(uint8_t outputMode) {
  return clampPicoOutputMode(outputMode) == kPicoOutputLfoFm ? kPicoLfoFmRateSteps
                                                            : kPicoLfoLoRateSteps;
}

inline uint8_t clampPicoLfoRateStep(int value, uint8_t outputMode) {
  return static_cast<uint8_t>(clampValue<int>(value, 0, picoLfoRateMaxStep(outputMode)));
}

inline uint8_t clampPicoLfoDepthPercent(int value) {
  int rounded = ((value + (kPicoLfoDepthStepPercent / 2)) / kPicoLfoDepthStepPercent) *
                kPicoLfoDepthStepPercent;
  return static_cast<uint8_t>(
      clampValue<int>(rounded, kPicoLfoMinDepthPercent, kPicoLfoMaxDepthPercent));
}

inline uint8_t picoLfoDepthIndexFromPercent(int percent) {
  uint8_t clamped = clampPicoLfoDepthPercent(percent);
  return static_cast<uint8_t>((clamped - kPicoLfoMinDepthPercent) / kPicoLfoDepthStepPercent);
}

inline uint8_t picoLfoDepthPercentFromIndex(int index) {
  return clampPicoLfoDepthPercent(
      kPicoLfoMinDepthPercent + clampValue<int>(index, 0, kPicoLfoDepthStepCount - 1) *
                                    kPicoLfoDepthStepPercent);
}

inline int8_t clampPicoLfoOffsetPercent(int value) {
  int step = static_cast<int>(kPicoLfoOffsetStepPercent);
  int rounded = ((value >= 0 ? value + step / 2 : value - step / 2) / step) * step;
  return static_cast<int8_t>(
      clampValue<int>(rounded, kPicoLfoMinOffsetPercent, kPicoLfoMaxOffsetPercent));
}

inline uint8_t picoLfoOffsetIndexFromPercent(int percent) {
  int8_t clamped = clampPicoLfoOffsetPercent(percent);
  return static_cast<uint8_t>((clamped - kPicoLfoMinOffsetPercent) / kPicoLfoOffsetStepPercent);
}

inline int8_t picoLfoOffsetPercentFromIndex(int index) {
  return clampPicoLfoOffsetPercent(
      kPicoLfoMinOffsetPercent + clampValue<int>(index, 0, kPicoLfoOffsetStepCount - 1) *
                                     kPicoLfoOffsetStepPercent);
}

inline uint8_t clampPicoLfoPulseWidth(int value) {
  int step = static_cast<int>(kPicoLfoPulseWidthStepPercent);
  int rounded = ((value + step / 2) / step) * step;
  return static_cast<uint8_t>(clampValue<int>(rounded,
                                              kPicoLfoMinPulseWidthPercent,
                                              kPicoLfoMaxPulseWidthPercent));
}

inline uint8_t picoLfoPulseWidthIndexFromPercent(int percent) {
  uint8_t clamped = clampPicoLfoPulseWidth(percent);
  return static_cast<uint8_t>((clamped - kPicoLfoMinPulseWidthPercent) /
                              kPicoLfoPulseWidthStepPercent);
}

inline uint8_t picoLfoPulseWidthPercentFromIndex(int index) {
  return clampPicoLfoPulseWidth(
      kPicoLfoMinPulseWidthPercent +
      clampValue<int>(index, 0, kPicoLfoPulseWidthStepCount - 1) *
          kPicoLfoPulseWidthStepPercent);
}

inline uint8_t clampPicoLfoLink(int value) {
  return static_cast<uint8_t>(clampValue<int>(value, 0, kPicoLfoLinkCount - 1));
}

inline uint8_t clampPicoLfoPhaseOffset(int value) {
  return static_cast<uint8_t>(clampValue<int>(value, 0, kPicoLfoPhaseOffsetCount - 1));
}

inline uint8_t clampPicoClockSource(int value) {
  return static_cast<uint8_t>(clampValue<int>(value, 0, kPicoClockSourceCount - 1));
}

// LFO2 rate as a ratio of LFO1: numerator over denominator.
inline uint8_t picoLfoLinkNumerator(uint8_t link) {
  switch (clampPicoLfoLink(link)) {
    case kPicoLfoLink3to2:
      return 3;
    case kPicoLfoLink2to1:
      return 2;
    case kPicoLfoLink4to1:
      return 4;
    default:
      return 1;
  }
}

inline uint8_t picoLfoLinkDenominator(uint8_t link) {
  switch (clampPicoLfoLink(link)) {
    case kPicoLfoLink1to2:
      return 2;
    case kPicoLfoLink1to4:
      return 4;
    case kPicoLfoLink3to2:
      return 2;
    default:
      return 1;
  }
}

inline const char* picoLfoLinkName(uint8_t link) {
  switch (clampPicoLfoLink(link)) {
    case kPicoLfoLink1to1:
      return "1:1";
    case kPicoLfoLink1to2:
      return "1:2";
    case kPicoLfoLink1to4:
      return "1:4";
    case kPicoLfoLink3to2:
      return "3:2";
    case kPicoLfoLink2to1:
      return "2:1";
    case kPicoLfoLink4to1:
      return "4:1";
    case kPicoLfoLinkOff:
    default:
      return "off";
  }
}

inline uint16_t picoLfoPhaseOffsetDegrees(uint8_t phaseOffset) {
  return static_cast<uint16_t>(clampPicoLfoPhaseOffset(phaseOffset)) * 90;
}

inline const char* picoClockSourceName(uint8_t source) {
  switch (clampPicoClockSource(source)) {
    case kPicoClockLfo1:
      return "lfo1";
    case kPicoClockLfo2:
      return "lfo2";
    case kPicoClockOff:
    default:
      return "off";
  }
}

inline const char* picoClockSourceDisplayLabel(uint8_t source) {
  switch (clampPicoClockSource(source)) {
    case kPicoClockLfo1:
      return "LFO1";
    case kPicoClockLfo2:
      return "LFO2";
    case kPicoClockOff:
    default:
      return "OFF";
  }
}

inline const char* picoOutputModeName(uint8_t mode) {
  switch (clampPicoOutputMode(mode)) {
    case kPicoOutputLfoLo:
      return "lo";
    case kPicoOutputLfoFm:
      return "fm";
    case kPicoOutputPedal:
    default:
      return "ped";
  }
}

inline const char* picoOutputModeDisplayLabel(uint8_t mode) {
  switch (clampPicoOutputMode(mode)) {
    case kPicoOutputLfoLo:
      return "LO";
    case kPicoOutputLfoFm:
      return "FM";
    case kPicoOutputPedal:
    default:
      return "PED";
  }
}

inline const char* picoLfoWaveName(uint8_t wave) {
  switch (clampPicoLfoWave(wave)) {
    case kPicoLfoTriangle:
      return "triangle";
    case kPicoLfoSawUp:
      return "sawup";
    case kPicoLfoSawDown:
      return "sawdown";
    case kPicoLfoSquare:
      return "square";
    case kPicoLfoPulse:
      return "pulse";
    case kPicoLfoSampleHold:
      return "sh";
    case kPicoLfoDrift:
      return "drift";
    case kPicoLfoSine:
    default:
      return "sine";
  }
}

inline const char* picoLfoWaveDisplayLabel(uint8_t wave) {
  switch (clampPicoLfoWave(wave)) {
    case kPicoLfoTriangle:
      return "TRI";
    case kPicoLfoSawUp:
      return "SAWUP";
    case kPicoLfoSawDown:
      return "SAWDN";
    case kPicoLfoSquare:
      return "SQR";
    case kPicoLfoPulse:
      return "PULS";
    case kPicoLfoSampleHold:
      return "SH";
    case kPicoLfoDrift:
      return "DRF";
    case kPicoLfoSine:
    default:
      return "SIN";
  }
}

inline float computePicoLfoRateHz(uint8_t outputMode, uint8_t rateStep) {
  uint8_t mode = clampPicoOutputMode(outputMode);
  float minHz = mode == kPicoOutputLfoFm ? kPicoLfoFmMinHz : kPicoLfoLoMinHz;
  float maxHz = mode == kPicoOutputLfoFm ? kPicoLfoFmMaxHz : kPicoLfoLoMaxHz;
  uint8_t maxStep = picoLfoRateMaxStep(mode);
  float t = maxStep == 0 ? 0.0f : static_cast<float>(clampPicoLfoRateStep(rateStep, mode)) /
                                      static_cast<float>(maxStep);
  return minHz * powf(maxHz / minHz, t);
}

inline float computePicoLfoRateHzForPedal(uint8_t outputMode, float pedal, uint8_t maxRateStep) {
  uint8_t mode = clampPicoOutputMode(outputMode);
  float minHz = mode == kPicoOutputLfoFm ? kPicoLfoFmMinHz : kPicoLfoLoMinHz;
  float maxHz = mode == kPicoOutputLfoFm ? kPicoLfoFmMaxHz : kPicoLfoLoMaxHz;
  uint8_t modeMaxStep = picoLfoRateMaxStep(mode);
  uint8_t clampedMaxStep = clampPicoLfoRateStep(maxRateStep, mode);
  float maxT = modeMaxStep == 0 ? 0.0f : static_cast<float>(clampedMaxStep) /
                                           static_cast<float>(modeMaxStep);
  float pedalT = clampValue(pedal, 0.0f, 1.0f);
  return minHz * powf(maxHz / minHz, pedalT * maxT);
}

inline uint8_t nearestPicoLfoRateStep(uint8_t outputMode, float hz) {
  uint8_t mode = clampPicoOutputMode(outputMode);
  float minHz = mode == kPicoOutputLfoFm ? kPicoLfoFmMinHz : kPicoLfoLoMinHz;
  float maxHz = mode == kPicoOutputLfoFm ? kPicoLfoFmMaxHz : kPicoLfoLoMaxHz;
  uint8_t maxStep = picoLfoRateMaxStep(mode);
  hz = clampValue(hz, minHz, maxHz);
  float t = logf(hz / minHz) / logf(maxHz / minHz);
  return clampPicoLfoRateStep(static_cast<int>(lroundf(t * maxStep)), mode);
}

inline float attenuatePicoLfoWaveValue(float value, uint8_t depthPercent) {
  float depth = static_cast<float>(clampPicoLfoDepthPercent(depthPercent)) / 100.0f;
  return clampValue(0.5f + (clampValue(value, 0.0f, 1.0f) - 0.5f) * depth, 0.0f, 1.0f);
}

inline float offsetPicoLfoWaveValue(float value, int8_t offsetPercent) {
  float offset = static_cast<float>(clampPicoLfoOffsetPercent(offsetPercent)) / 100.0f;
  return clampValue(clampValue(value, 0.0f, 1.0f) + offset, 0.0f, 1.0f);
}

// Deterministic per-cycle random in [0, 1); same (cycle, seed) always yields the
// same value, so S+H and drift stay stable across control-loop reruns.
inline float picoLfoRandomUnit(uint32_t cycle, uint32_t seed) {
  uint32_t x = cycle * 0x9E3779B9u ^ (seed + 0x85EBCA6Bu);
  x ^= x >> 16;
  x *= 0x7FEB352Du;
  x ^= x >> 15;
  x *= 0x846CA68Bu;
  x ^= x >> 16;
  return static_cast<float>(x >> 8) * (1.0f / 16777216.0f);
}

inline float computePicoLfoWaveValue(float phase,
                                     uint8_t wave,
                                     uint32_t cycle = 0,
                                     uint32_t seed = 0,
                                     uint8_t pulseWidthPercent = kPicoDefaultLfoPulseWidthPercent) {
  phase = phase - floorf(phase);
  if (phase < 0.0f) {
    phase += 1.0f;
  }

  switch (clampPicoLfoWave(wave)) {
    case kPicoLfoTriangle:
      return phase < 0.5f ? phase * 2.0f : (1.0f - phase) * 2.0f;
    case kPicoLfoSawUp:
      return phase;
    case kPicoLfoSawDown:
      return 1.0f - phase;
    case kPicoLfoSquare:
      return phase < 0.5f ? 1.0f : 0.0f;
    case kPicoLfoPulse:
      return phase < static_cast<float>(clampPicoLfoPulseWidth(pulseWidthPercent)) / 100.0f
                 ? 1.0f
                 : 0.0f;
    case kPicoLfoSampleHold:
      return picoLfoRandomUnit(cycle, seed);
    case kPicoLfoDrift: {
      float from = picoLfoRandomUnit(cycle, seed);
      float to = picoLfoRandomUnit(cycle + 1u, seed);
      float t = phase * phase * (3.0f - 2.0f * phase);
      return from + (to - from) * t;
    }
    case kPicoLfoSine:
    default:
      return 0.5f + 0.5f * sinf(phase * 6.28318530718f);
  }
}

// Derive LFO2's phase and cycle from LFO1 when linked, so the pair never
// drifts. The ratio is numerator:denominator of LFO2 rate vs LFO1 rate and the
// phase offset shifts LFO2 by quarter cycles.
inline void computePicoLinkedPhase(float sourcePhase,
                                   uint32_t sourceCycle,
                                   uint8_t link,
                                   uint8_t phaseOffset,
                                   float* outPhase,
                                   uint32_t* outCycle) {
  uint8_t num = picoLfoLinkNumerator(link);
  uint8_t den = picoLfoLinkDenominator(link);
  sourcePhase = clampValue(sourcePhase - floorf(sourcePhase), 0.0f, 1.0f);

  float posInBlock = static_cast<float>(sourceCycle % den) + sourcePhase;
  float scaled = (posInBlock * static_cast<float>(num)) / static_cast<float>(den) +
                 static_cast<float>(clampPicoLfoPhaseOffset(phaseOffset)) * 0.25f;
  float wholeInBlock = floorf(scaled);

  if (outPhase != nullptr) {
    *outPhase = scaled - wholeInBlock;
  }
  if (outCycle != nullptr) {
    *outCycle = (sourceCycle / den) * num + static_cast<uint32_t>(wholeInBlock);
  }
}

inline const char* picoCurveName(uint8_t mode) {
  switch (clampPicoCurveMode(mode)) {
    case kPicoCurveEaseOut:
      return "easeout";
    case kPicoCurveSquare:
      return "square";
    case kPicoCurveSmooth:
      return "smooth";
    case kPicoCurveLinear:
    default:
      return "linear";
  }
}

inline const char* picoCurveDisplayLabel(uint8_t mode) {
  switch (clampPicoCurveMode(mode)) {
    case kPicoCurveEaseOut:
      return "EOUT";
    case kPicoCurveSquare:
      return "SQR";
    case kPicoCurveSmooth:
      return "SMTH";
    case kPicoCurveLinear:
    default:
      return "LIN";
  }
}

inline int32_t centsToMicrovolts(float cents, float centsPerVolt = 1200.0f) {
  if (centsPerVolt <= 0.0f) {
    return 0;
  }
  return static_cast<int32_t>(lroundf((cents * 1000000.0f) / centsPerVolt));
}

inline int8_t clampPicoSemitones(int value) {
  return static_cast<int8_t>(clampValue<int>(value, kPicoMinSemitones, kPicoMaxSemitones));
}

inline uint8_t picoSemitoneMapIndex(int semitones) {
  return static_cast<uint8_t>(clampPicoSemitones(semitones) - kPicoMinSemitones);
}

inline uint8_t picoIntervalMagnitude(int semitones) {
  int clamped = clampPicoSemitones(semitones);
  return static_cast<uint8_t>(clamped < 0 ? -clamped : clamped);
}

inline int8_t signedPicoInterval(uint8_t magnitude, PicoBendDirection direction) {
  int clamped = clampValue<int>(magnitude, 0, kPicoMaxSemitones);
  return static_cast<int8_t>(direction == kPicoBendDown ? -clamped : clamped);
}

inline int32_t picoSemitonesToMicrovolts(int semitones,
                                         int32_t octaveMicrovolts = kDefaultOctaveMicrovolts) {
  int clamped = clampPicoSemitones(semitones);
  return static_cast<int32_t>(
      lroundf((static_cast<float>(clamped) * static_cast<float>(octaveMicrovolts)) / 12.0f));
}

inline int32_t computePicoLinearToeMicrovolts(
    int semitones,
    int32_t centerMicrovolts = kDefaultCenterMicrovolts,
    int32_t octaveMicrovolts = kDefaultOctaveMicrovolts) {
  return clampValue<int32_t>(
      centerMicrovolts + picoSemitonesToMicrovolts(semitones, octaveMicrovolts),
      0,
      kPicoDacFullScaleMicrovolts);
}

inline int32_t picoUnipolarNoBendMicrovolts(PicoBendDirection direction) {
  return direction == kPicoBendDown ? kPicoDacFullScaleMicrovolts : 0;
}

inline int32_t computePicoUnipolarToeMicrovolts(
    int semitones,
    int32_t octaveMicrovolts = kDefaultUnipolarOctaveMicrovolts) {
  uint8_t magnitude = picoIntervalMagnitude(semitones);
  int32_t clampedOctave =
      clampValue<int32_t>(octaveMicrovolts, 0, kPicoDacFullScaleMicrovolts);
  int32_t delta = static_cast<int32_t>(
      lroundf((static_cast<float>(magnitude) * static_cast<float>(clampedOctave)) / 12.0f));
  if (clampPicoSemitones(semitones) < 0) {
    return clampValue<int32_t>(kPicoDacFullScaleMicrovolts - delta,
                               0,
                               kPicoDacFullScaleMicrovolts);
  }
  return clampValue<int32_t>(delta, 0, kPicoDacFullScaleMicrovolts);
}

inline int32_t computePicoResponseToeMicrovolts(
    int semitones,
    uint16_t fullScaleResponseCents,
    int32_t fullScaleMicrovolts = kPicoDacFullScaleMicrovolts) {
  int clamped = clampPicoSemitones(semitones);
  if (clamped == 0) {
    return clamped < 0 ? fullScaleMicrovolts : 0;
  }
  if (fullScaleResponseCents == 0 || fullScaleMicrovolts <= 0) {
    return computePicoUnipolarToeMicrovolts(clamped, fullScaleMicrovolts);
  }

  int32_t targetCents = static_cast<int32_t>(picoIntervalMagnitude(clamped)) * 100L;
  int64_t excursion = (static_cast<int64_t>(targetCents) * fullScaleMicrovolts +
                       (fullScaleResponseCents / 2)) /
                      fullScaleResponseCents;
  int32_t clampedExcursion =
      clampValue<int32_t>(static_cast<int32_t>(excursion), 0, fullScaleMicrovolts);

  if (clamped < 0) {
    return clampValue<int32_t>(fullScaleMicrovolts - clampedExcursion,
                               0,
                               fullScaleMicrovolts);
  }
  return clampedExcursion;
}

inline int32_t computePicoMappedOutputMicrovolts(float pedal,
                                                 int32_t centerMicrovolts,
                                                 int32_t toeMicrovolts) {
  pedal = clampValue(pedal, 0.0f, 1.0f);
  int32_t output = centerMicrovolts +
                   static_cast<int32_t>(
                       lroundf(static_cast<float>(toeMicrovolts - centerMicrovolts) * pedal));
  return clampValue<int32_t>(output, 0, kPicoDacFullScaleMicrovolts);
}

inline int32_t computePicoCenteredOutputMicrovolts(
    float pedal,
    int semitones,
    int32_t centerMicrovolts = kDefaultCenterMicrovolts,
    int32_t octaveMicrovolts = kDefaultOctaveMicrovolts) {
  return computePicoMappedOutputMicrovolts(
      pedal,
      centerMicrovolts,
      computePicoLinearToeMicrovolts(semitones, centerMicrovolts, octaveMicrovolts));
}

inline uint16_t microvoltsToMcp4728Code(int32_t microvolts,
                                        int32_t fullScaleMicrovolts = kPicoDacFullScaleMicrovolts) {
  if (fullScaleMicrovolts <= 0) {
    return 0;
  }

  int64_t clamped = clampValue<int64_t>(microvolts, 0, fullScaleMicrovolts);
  int64_t code = (clamped * kMcp4728MaxCode + (fullScaleMicrovolts / 2)) /
                 fullScaleMicrovolts;
  return static_cast<uint16_t>(clampValue<int64_t>(code, 0, kMcp4728MaxCode));
}

inline const char* picoAbsoluteIntervalLabel(uint8_t semitones) {
  switch (semitones) {
    case 0:
      return "1";
    case 1:
      return "b2";
    case 2:
      return "2";
    case 3:
      return "b3";
    case 4:
      return "3";
    case 5:
      return "4";
    case 6:
      return "b5";
    case 7:
      return "5";
    case 8:
      return "b6";
    case 9:
      return "6";
    default:
      return "?";
  }
}

struct PedalCalibration {
  uint16_t heelRaw = 0;
  uint16_t toeRaw = 4095;
  bool invert = false;
  float snapLow = 0.005f;
  float snapHigh = 0.995f;
  float deadband = 0.0005f;
};

struct PedalFilterSettings {
  bool enabled = true;
  float minCutoffHz = 3.5f;
  float beta = 0.12f;
  float derivativeCutoffHz = 1.0f;
};

struct PedalState {
  uint16_t raw = 0;
  float normalized = 0.0f;
  float filtered = 0.0f;
  float curved = 0.0f;
  bool snappedHeel = true;
  bool snappedToe = false;
};

class PedalProcessor {
 public:
  void configure(const PedalCalibration& calibration,
                 const PedalFilterSettings& filterSettings,
                 uint8_t curveModeValue) {
    cal = calibration;
    filterConfig = filterSettings;
    curveMode = curveModeValue;
  }

  void reset() {
    previousDeadbanded = 0.0f;
    previousFiltered = 0.0f;
    hasPrevious = false;
  }

  PedalState process(uint16_t raw, float sampleRateHz) {
    PedalState state;
    state.raw = raw;

    float normalized = normalize(raw);
    if (hasPrevious && fabsf(normalized - previousDeadbanded) < cal.deadband) {
      normalized = previousDeadbanded;
    } else {
      previousDeadbanded = normalized;
    }

    if (!hasPrevious) {
      previousFiltered = normalized;
      hasPrevious = true;
    }

    state.normalized = normalized;
    if (filterConfig.enabled) {
      float alpha = filterAlpha(sampleRateHz);
      previousFiltered += alpha * (normalized - previousFiltered);
      state.filtered = previousFiltered;
    } else {
      state.filtered = normalized;
    }
    state.filtered = clampValue(state.filtered, 0.0f, 1.0f);

    float snapped = state.filtered;
    state.snappedHeel = false;
    state.snappedToe = false;
    if (snapped <= cal.snapLow) {
      snapped = 0.0f;
      state.snappedHeel = true;
    } else if (snapped >= cal.snapHigh) {
      snapped = 1.0f;
      state.snappedToe = true;
    }

    state.curved = applyCurve(snapped);
    return state;
  }

 private:
  float normalize(uint16_t raw) const {
    float heel = static_cast<float>(cal.heelRaw);
    float toe = static_cast<float>(cal.toeRaw);
    float span = toe - heel;

    if (fabsf(span) < 8.0f) {
      return 0.0f;
    }

    float value = (static_cast<float>(raw) - heel) / span;
    value = clampValue(value, 0.0f, 1.0f);
    if (cal.invert) {
      value = 1.0f - value;
    }
    return value;
  }

  float filterAlpha(float sampleRateHz) const {
    if (sampleRateHz <= 0.0f || filterConfig.minCutoffHz <= 0.0f) {
      return 1.0f;
    }
    float tau = 1.0f / (2.0f * 3.14159265f * filterConfig.minCutoffHz);
    float dt = 1.0f / sampleRateHz;
    float alpha = dt / (tau + dt);
    return clampValue(alpha + filterConfig.beta, 0.02f, 1.0f);
  }

  float applyCurve(float x) const {
    x = clampValue(x, 0.0f, 1.0f);
    switch (curveMode) {
      case 1:
        return sqrtf(x);
      case 2:
        return x * x;
      case 3:
        return x * x * (3.0f - 2.0f * x);
      case 0:
      default:
        return x;
    }
  }

  PedalCalibration cal;
  PedalFilterSettings filterConfig;
  uint8_t curveMode = 0;
  float previousDeadbanded = 0.0f;
  float previousFiltered = 0.0f;
  bool hasPrevious = false;
};

} // namespace expctrl
