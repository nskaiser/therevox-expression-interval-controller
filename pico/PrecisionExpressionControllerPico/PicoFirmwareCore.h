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
constexpr uint16_t kMcp4725MaxCode = 4095;
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
constexpr uint8_t kPicoLfoMinDepthPercent = 50;
constexpr uint8_t kPicoLfoMaxDepthPercent = 100;
constexpr uint8_t kPicoLfoDepthStepPercent = 5;
constexpr uint8_t kPicoLfoDepthStepCount =
    ((kPicoLfoMaxDepthPercent - kPicoLfoMinDepthPercent) / kPicoLfoDepthStepPercent) + 1;

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
  kPicoLfoWaveCount = 6,
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

inline float computePicoLfoWaveValue(float phase, uint8_t wave) {
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
      return phase < 0.25f ? 1.0f : 0.0f;
    case kPicoLfoSine:
    default:
      return 0.5f + 0.5f * sinf(phase * 6.28318530718f);
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

inline uint16_t microvoltsToMcp4725Code(int32_t microvolts,
                                        int32_t fullScaleMicrovolts = kPicoDacFullScaleMicrovolts) {
  if (fullScaleMicrovolts <= 0) {
    return 0;
  }

  int64_t clamped = clampValue<int64_t>(microvolts, 0, fullScaleMicrovolts);
  int64_t code = (clamped * kMcp4725MaxCode + (fullScaleMicrovolts / 2)) /
                 fullScaleMicrovolts;
  return static_cast<uint16_t>(clampValue<int64_t>(code, 0, kMcp4725MaxCode));
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
