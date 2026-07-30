// Therevox Expression Controller - Raspberry Pi Pico H prototype
//
// Hardware target:
//   Raspberry Pi Pico H
//   MCP4728 quad DAC breakout, I2C address 0x60
//   Adafruit 0.91" 128x32 SSD1306 OLED, I2C address 0x3C
//   Adafruit rotary encoder + push switch
//   Two 1/4" TRS jacks: expression pedal input and Therevox expression/CV output
//   Two 3.5mm TS jacks: dedicated Therevox patch-panel LFO outputs
//
// Exact Pico H wiring:
//   Expression input TRS Ring   -> Pico pin labeled 3V3(OUT), physical pin 36
//   Expression input TRS Sleeve -> Pico pin labeled GND, physical pin 38
//   Expression input TRS Tip    -> 1k resistor -> Pico pin labeled GP26/ADC0, physical pin 31
//   GP26/ADC0 side of 1k       -> 100nF capacitor -> GND
//
//   MCP4728 VCC                 -> Pico pin labeled 3V3(OUT), physical pin 36
//   MCP4728 GND                 -> Pico pin labeled GND, physical pin 38
//   MCP4728 SDA                 -> Pico pin labeled GP4, physical pin 6
//   MCP4728 SCL                 -> Pico pin labeled GP5, physical pin 7
//   MCP4728 LDAC                -> GND (required; outputs freeze if LDAC floats high)
//   MCP4728 VOUTA               -> 1k resistor -> output plug physical Tip
//   Output plug physical Ring   -> not connected for active external CV
//   Output TRS Sleeve           -> GND
//   MCP4728 VOUTB               -> 1k resistor -> LFO 1 3.5mm jack Tip
//   MCP4728 VOUTC               -> 1k resistor -> LFO 2 3.5mm jack Tip
//   MCP4728 VOUTD               -> 1k resistor -> optional clock 3.5mm jack Tip
//   LFO 1/2/clock jack Sleeve   -> GND
//
//   The firmware uses MCP4728 multi-write commands (UDAC=0), which set
//   VREF=VDD and gain=1 on every update and latch outputs regardless of the
//   LDAC pin state. Grounding LDAC is still recommended.
//
//   OLED red/VCC                -> Pico pin labeled 3V3(OUT), physical pin 36
//   OLED black/GND              -> Pico pin labeled GND, physical pin 38
//   OLED blue/SDA               -> Pico pin labeled GP4, physical pin 6
//   OLED yellow/SCL             -> Pico pin labeled GP5, physical pin 7
//
//   Encoder C/common            -> GND
//   Encoder A/CLK               -> Pico pin labeled GP14, physical pin 19
//   Encoder B/DT                -> Pico pin labeled GP15, physical pin 20
//   Encoder switch lug          -> Pico pin labeled GP13, physical pin 17
//   Encoder other switch lug    -> GND
//
// Control model:
//   PED mode: expression pedal controls calibrated interval bend.
//   UP mode:   heel/no-bend 0.000 V, top interval +6 / 9 semitones.
//   DOWN mode: heel/no-bend 3.300 V, bottom interval -6 / -9 semitones.
//   LO mode: slow unipolar LFO, 0.05-20 Hz.
//   FM mode: faster unipolar LFO, 8-160 Hz.
//   In LO/FM, the expression pedal sweeps LFO speed and the encoder sets depth.
//   LFO 1 and LFO 2 are independent dedicated patch-panel CV outputs.
//   LFO 2 can link to LFO 1 at a rate ratio with a quarter-cycle phase offset.
//   VOUTD can emit a full-swing clock square derived from LFO 1 or LFO 2.
//   Double-click the encoder to cycle the active edit target: EXP, LFO1, LFO2.
//   Repeated single presses on a focused LFO tap-tempo its rate.
//
// Tune/calibrate the Therevox with the controller connected and the pedal at heel.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__has_include)
#if __has_include(<EEPROM.h>)
#include <EEPROM.h>
#define EXPCTRL_PICO_HAS_EEPROM 1
#endif
#endif

#ifndef EXPCTRL_PICO_HAS_EEPROM
#define EXPCTRL_PICO_HAS_EEPROM 0
#endif

#if EXPCTRL_PICO_HAS_EEPROM && !defined(ARDUINO_ARCH_MBED)
#define EXPCTRL_PICO_USE_EEPROM 1
#else
#define EXPCTRL_PICO_USE_EEPROM 0
#endif

#include "PicoFirmwareCore.h"

using namespace expctrl;

namespace {

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint8_t PEDAL_PIN = A0;          // Pico GP26/ADC0, physical pin 31.
constexpr uint8_t I2C_SDA_PIN = 4;         // Pico GP4, physical pin 6.
constexpr uint8_t I2C_SCL_PIN = 5;         // Pico GP5, physical pin 7.
constexpr uint8_t ENCODER_SWITCH_PIN = 13; // Pico GP13, physical pin 17.
constexpr uint8_t ENCODER_A_PIN = 14;      // Pico GP14, physical pin 19.
constexpr uint8_t ENCODER_B_PIN = 15;      // Pico GP15, physical pin 20.
constexpr uint8_t MCP4728_ADDRESS = 0x60;
constexpr uint8_t SSD1306_ADDRESS = 0x3C;
constexpr uint8_t LFO_OUT_COUNT = 2;
constexpr uint8_t DAC_CHANNEL_COUNT = 4;
constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_MAX = (1u << ADC_BITS) - 1u;
constexpr uint8_t PEDAL_OVERSAMPLES = 8;
constexpr uint32_t CONTROL_PERIOD_US = 1000;
constexpr uint32_t MONITOR_MS = 500;
constexpr uint32_t DISPLAY_MS = 500;
constexpr uint32_t HEARTBEAT_MS = 1000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t BUTTON_DOUBLE_CLICK_MS = 400;
constexpr uint32_t MENU_LONG_PRESS_MS = 2000;
constexpr uint32_t CAL_MESSAGE_MS = 1500;
constexpr uint32_t CAL_PROMPT_MS = 60UL * 1000UL;
constexpr bool IDLE_SLEEP_ENABLED = false;
constexpr uint32_t IDLE_SLEEP_MS = 60UL * 60UL * 1000UL;
constexpr uint32_t SETTINGS_AUTOSAVE_MS = 3000;
constexpr uint32_t DAC_PEDAL_WRITE_MIN_INTERVAL_US = 5000;
constexpr uint32_t DAC_LFO_WRITE_MIN_INTERVAL_US = 1000;
constexpr uint16_t DAC_CODE_DEADBAND = 2;
constexpr uint16_t PEDAL_ACTIVITY_RAW_DELTA = 8;
constexpr uint16_t MIN_CAL_SPAN_RAW = 128;
constexpr float PEDAL_SNAP_LOW = 0.005f;
constexpr float PEDAL_SNAP_HIGH = 0.995f;
constexpr float PEDAL_DEADBAND = 0.0005f;
constexpr uint8_t DISPLAY_WIDTH = 128;
constexpr uint8_t DISPLAY_HEIGHT = 32;
constexpr uint16_t DISPLAY_BUFFER_SIZE = DISPLAY_WIDTH * DISPLAY_HEIGHT / 8;
constexpr uint32_t SETTINGS_MAGIC = 0x5049434FUL; // PICO
constexpr uint16_t SETTINGS_VERSION = 12;
constexpr uint16_t SETTINGS_VERSION_PREVIOUS = 11;
constexpr uint32_t TAP_MIN_MS = 450;
constexpr uint32_t TAP_MAX_MS = 5000;
constexpr uint16_t DEFAULT_RANGE_MV = static_cast<uint16_t>(kDefaultUnipolarOctaveMicrovolts / 1000L);
constexpr uint16_t MAX_OUTPUT_MV = static_cast<uint16_t>(kPicoDacFullScaleMicrovolts / 1000L);
constexpr uint16_t MAX_OCTAVE_SCALE_MV = MAX_OUTPUT_MV;
constexpr uint16_t DEFAULT_TUNE_STEP_MV = 10;
constexpr uint16_t MAX_TUNE_STEP_MV = 250;

enum OutputFocus : uint8_t {
  OUTPUT_FOCUS_EXP = 0,
  OUTPUT_FOCUS_LFO1 = 1,
  OUTPUT_FOCUS_LFO2 = 2,
  OUTPUT_FOCUS_COUNT = 3,
};

struct PicoSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint16_t heelRaw;
  uint16_t toeRaw;
  int8_t semitones;
  uint8_t invert;
  uint8_t monitorEnabled;
  uint8_t bendDirection;
  uint8_t curveMode;
  uint8_t outputMode;
  uint8_t lfoWave;
  uint8_t lfoLoRate;
  uint8_t lfoFmRate;
  uint8_t lfoDepthPercent;
  uint8_t outputFocus;
  uint8_t lfoOutMode[LFO_OUT_COUNT];
  uint8_t lfoOutWave[LFO_OUT_COUNT];
  uint8_t lfoOutLoRate[LFO_OUT_COUNT];
  uint8_t lfoOutFmRate[LFO_OUT_COUNT];
  uint8_t lfoOutDepthPercent[LFO_OUT_COUNT];
  uint8_t lfoOutPolarity[LFO_OUT_COUNT];
  uint8_t lfoPulseWidth;
  uint8_t lfoOutPulseWidth[LFO_OUT_COUNT];
  int8_t lfoOutOffsetPercent[LFO_OUT_COUNT];
  uint8_t lfo2Link;
  uint8_t lfo2PhaseOffset;
  uint8_t clockSource;
  uint16_t rangeMv;
  uint16_t responseCents;
  uint16_t toeMapMv[kPicoSemitoneMapCount];
  uint32_t checksum;
};

// Exact on-flash layout of settings version 11, kept so an upgrade preserves
// calibration and the hand-tuned toe map instead of wiping them.
struct PicoSettingsV11 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint16_t heelRaw;
  uint16_t toeRaw;
  int8_t semitones;
  uint8_t invert;
  uint8_t monitorEnabled;
  uint8_t bendDirection;
  uint8_t curveMode;
  uint8_t outputMode;
  uint8_t lfoWave;
  uint8_t lfoLoRate;
  uint8_t lfoFmRate;
  uint8_t lfoDepthPercent;
  uint8_t outputFocus;
  uint8_t lfoOutMode[LFO_OUT_COUNT];
  uint8_t lfoOutWave[LFO_OUT_COUNT];
  uint8_t lfoOutLoRate[LFO_OUT_COUNT];
  uint8_t lfoOutFmRate[LFO_OUT_COUNT];
  uint8_t lfoOutDepthPercent[LFO_OUT_COUNT];
  uint8_t lfoOutPolarity[LFO_OUT_COUNT];
  uint16_t rangeMv;
  uint16_t responseCents;
  uint16_t toeMapMv[kPicoSemitoneMapCount];
  uint32_t checksum;
};

enum CalibrationState {
  CAL_IDLE,
  CAL_WAIT_HEEL,
  CAL_WAIT_TOE,
};

enum MenuItem : uint8_t {
  MENU_OUT = 0,
  MENU_MODE = 1,
  MENU_WAVE = 2,
  MENU_DEPTH = 3,
  MENU_PW = 4,
  MENU_OFS = 5,
  MENU_LINK = 6,
  MENU_PHS = 7,
  MENU_CLK = 8,
  MENU_CURVE = 9,
  MENU_CAL = 10,
  MENU_DIR = 11,
  MENU_DONE = 12,
  MENU_COUNT = 13,
};

enum MenuEditField : uint8_t {
  MENU_EDIT_NONE = 0,
  MENU_EDIT_OUT,
  MENU_EDIT_MODE,
  MENU_EDIT_WAVE,
  MENU_EDIT_DEPTH,
  MENU_EDIT_PW,
  MENU_EDIT_OFS,
  MENU_EDIT_LINK,
  MENU_EDIT_PHS,
  MENU_EDIT_CLK,
  MENU_EDIT_CURVE,
};

uint16_t microvoltsToMillivolts(int32_t microvolts) {
  return static_cast<uint16_t>(
      clampValue<int32_t>((microvolts + 500L) / 1000L, 0, MAX_OUTPUT_MV));
}

uint16_t linearToeMillivoltsForSemitone(int semitones, uint16_t rangeMv) {
  return microvoltsToMillivolts(
      computePicoUnipolarToeMicrovolts(semitones,
                                       static_cast<int32_t>(rangeMv) * 1000L));
}

void fillLinearToeMap(PicoSettings& value) {
  for (int semitone = kPicoMinSemitones; semitone <= kPicoMaxSemitones; ++semitone) {
    value.toeMapMv[picoSemitoneMapIndex(semitone)] =
        linearToeMillivoltsForSemitone(semitone, value.rangeMv);
  }
  value.toeMapMv[picoSemitoneMapIndex(0)] = 0;
}

uint16_t responseToeMillivoltsForSemitone(int semitones, uint16_t responseCents) {
  return microvoltsToMillivolts(
      computePicoResponseToeMicrovolts(semitones, responseCents));
}

void fillResponseToeMap(PicoSettings& value) {
  for (int semitone = kPicoMinSemitones; semitone <= kPicoMaxSemitones; ++semitone) {
    value.toeMapMv[picoSemitoneMapIndex(semitone)] =
        responseToeMillivoltsForSemitone(semitone, value.responseCents);
  }
  value.toeMapMv[picoSemitoneMapIndex(0)] = 0;
}

void fillToeMapFromCurrentFit(PicoSettings& value) {
  if (value.responseCents > 0) {
    fillResponseToeMap(value);
  } else {
    fillLinearToeMap(value);
  }
}

bool toeMapValid(const PicoSettings& value) {
  for (uint8_t i = 0; i < kPicoSemitoneMapCount; ++i) {
    if (value.toeMapMv[i] > MAX_OUTPUT_MV) {
      return false;
    }
  }
  return true;
}

PicoSettings settings;
PedalProcessor pedalProcessor;
PedalState pedalState;
CalibrationState calState = CAL_IDLE;

uint16_t currentRaw = 0;
uint16_t lastActivityRaw = 0;
int32_t currentOutputMicrovolts = 0;
int32_t lfoOutOutputMicrovolts[LFO_OUT_COUNT] = {0, 0};
uint16_t currentDacCode = 0;
uint16_t lfoOutDacCode[LFO_OUT_COUNT] = {0, 0};
int lastWrittenDacCodes[DAC_CHANNEL_COUNT] = {-1, -1, -1, -1};
bool dacReady = false;
bool displayReady = false;
bool sleeping = false;
bool settingsDirty = false;
bool tuneMode = false;
bool cvOverrideEnabled = false;
uint16_t cvOverrideMv = 0;
uint16_t tuneStepMv = DEFAULT_TUNE_STEP_MV;

uint32_t lastControlUs = 0;
uint32_t lastDacWriteUs = 0;
uint32_t lastMonitorMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastActivityMs = 0;
uint32_t lastSettingsChangeMs = 0;
uint32_t displayMessageUntilMs = 0;

uint8_t displayBuffer[DISPLAY_BUFFER_SIZE];
char displayMessageTop[16] = "";
char displayMessageBottom[24] = "";
char serialLine[96];
size_t serialLineLength = 0;

bool displayForceUpdate = true;
bool lastRenderedMessage = false;
int lastRenderedCalState = -1;
int lastRenderedSemitones = 999;
bool lastRenderedDacReady = false;
bool lastRenderedSettingsDirty = false;
bool lastRenderedTuneMode = false;
bool lastRenderedCvOverrideEnabled = false;
uint16_t lastRenderedCvOverrideMv = 0xffff;
bool lastRenderedMenuActive = false;
uint8_t lastRenderedMenuIndex = 0xff;
uint8_t lastRenderedMenuEditField = 0xff;
uint8_t lastRenderedMenuEditValue = 0xff;
uint8_t lastRenderedCurveMode = 0xff;
uint8_t lastRenderedBendDirection = 0xff;
uint8_t lastRenderedOutputFocus = 0xff;
uint8_t lastRenderedOutputMode = 0xff;
uint8_t lastRenderedLfoWave = 0xff;
uint8_t lastRenderedLfoLoRate = 0xff;
uint8_t lastRenderedLfoFmRate = 0xff;
uint8_t lastRenderedLfoDepthPercent = 0xff;
uint8_t lastRenderedLfoOutMode[LFO_OUT_COUNT] = {0xff, 0xff};
uint8_t lastRenderedLfoOutWave[LFO_OUT_COUNT] = {0xff, 0xff};
uint8_t lastRenderedLfoOutLoRate[LFO_OUT_COUNT] = {0xff, 0xff};
uint8_t lastRenderedLfoOutFmRate[LFO_OUT_COUNT] = {0xff, 0xff};
uint8_t lastRenderedLfoOutDepthPercent[LFO_OUT_COUNT] = {0xff, 0xff};
uint8_t lastRenderedLfoOutPolarity[LFO_OUT_COUNT] = {0xff, 0xff};
uint8_t lastRenderedLfoOutPulseWidth[LFO_OUT_COUNT] = {0xff, 0xff};
int8_t lastRenderedLfoOutOffsetPercent[LFO_OUT_COUNT] = {-128, -128};
uint8_t lastRenderedLfoPulseWidth = 0xff;
uint8_t lastRenderedLfo2Link = 0xff;
uint8_t lastRenderedLfo2PhaseOffset = 0xff;
uint8_t lastRenderedClockSource = 0xff;

bool menuActive = false;
uint8_t menuIndex = MENU_OUT;
uint8_t menuEditField = MENU_EDIT_NONE;
uint8_t menuEditValue = 0;
float lfoPhase = 0.0f;
uint32_t lfoCycle = 0;
uint32_t lastLfoUs = 0;
float lfoOutPhase[LFO_OUT_COUNT] = {0.0f, 0.0f};
uint32_t lfoOutCycle[LFO_OUT_COUNT] = {0, 0};
uint32_t lastLfoOutUs[LFO_OUT_COUNT] = {0, 0};
uint16_t clockDacCode = 0;
uint32_t lastTapMs = 0;
uint8_t lastTapFocus = 0xff;

// Fixed hash seeds so each output gets its own S+H/drift random sequence.
constexpr uint32_t LFO_SEED_EXP = 0xE0u;
constexpr uint32_t LFO_SEED_OUT[LFO_OUT_COUNT] = {0xA1u, 0xB2u};

volatile uint8_t lastEncoderState = 0;
volatile int8_t encoderTransitionCount = 0;
volatile int8_t encoderPendingSteps = 0;
volatile uint32_t encoderTransitionEvents = 0;
bool encoderDebug = false;
bool buttonDebug = false;

bool lastButtonRawPressed = false;
bool stableButtonPressed = false;
bool buttonReleasedSinceBoot = false;
uint32_t lastButtonChangeMs = 0;
uint32_t buttonPressStartedMs = 0;
bool buttonLongHandled = false;
bool buttonIgnoreRelease = false;
bool pendingShortPress = false;
uint32_t pendingShortPressMs = 0;

uint32_t fnv1a(const uint8_t* data, size_t length) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < length; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

uint32_t settingsChecksum(const PicoSettings& value) {
  return fnv1a(reinterpret_cast<const uint8_t*>(&value), offsetof(PicoSettings, checksum));
}

void finalizeSettings(PicoSettings& value) {
  value.magic = SETTINGS_MAGIC;
  value.version = SETTINGS_VERSION;
  value.size = sizeof(PicoSettings);
  value.checksum = settingsChecksum(value);
}

bool settingsValid(const PicoSettings& value) {
  if (!(value.magic == SETTINGS_MAGIC &&
        value.version == SETTINGS_VERSION &&
        value.size == sizeof(PicoSettings) &&
        value.checksum == settingsChecksum(value) &&
        value.heelRaw <= ADC_MAX &&
        value.toeRaw <= ADC_MAX &&
        value.rangeMv <= MAX_OCTAVE_SCALE_MV &&
        value.responseCents <= 6000 &&
        value.bendDirection <= static_cast<uint8_t>(kPicoBendDown) &&
        value.curveMode < kPicoCurveCount &&
        value.outputMode < kPicoOutputModeCount &&
        value.lfoWave < kPicoLfoWaveCount &&
        value.lfoLoRate <= kPicoLfoLoRateSteps &&
        value.lfoFmRate <= kPicoLfoFmRateSteps &&
        value.lfoDepthPercent >= kPicoLfoMinDepthPercent &&
        value.lfoDepthPercent <= kPicoLfoMaxDepthPercent &&
        ((value.lfoDepthPercent - kPicoLfoMinDepthPercent) % kPicoLfoDepthStepPercent == 0) &&
        value.outputFocus < OUTPUT_FOCUS_COUNT &&
        toeMapValid(value) &&
        value.semitones >= kPicoMinSemitones &&
        value.semitones <= kPicoMaxSemitones &&
        ((value.bendDirection == static_cast<uint8_t>(kPicoBendDown) && value.semitones <= 0) ||
         (value.bendDirection == static_cast<uint8_t>(kPicoBendUp) && value.semitones >= 0)))) {
    return false;
  }

  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    if (value.lfoOutMode[i] != kPicoOutputLfoLo &&
        value.lfoOutMode[i] != kPicoOutputLfoFm) {
      return false;
    }
    if (value.lfoOutWave[i] >= kPicoLfoWaveCount ||
        value.lfoOutLoRate[i] > kPicoLfoLoRateSteps ||
        value.lfoOutFmRate[i] > kPicoLfoFmRateSteps ||
        value.lfoOutDepthPercent[i] < kPicoLfoMinDepthPercent ||
        value.lfoOutDepthPercent[i] > kPicoLfoMaxDepthPercent ||
        ((value.lfoOutDepthPercent[i] - kPicoLfoMinDepthPercent) %
             kPicoLfoDepthStepPercent !=
         0) ||
        value.lfoOutPolarity[i] > 1) {
      return false;
    }
    if (value.lfoOutPulseWidth[i] < kPicoLfoMinPulseWidthPercent ||
        value.lfoOutPulseWidth[i] > kPicoLfoMaxPulseWidthPercent ||
        value.lfoOutOffsetPercent[i] < kPicoLfoMinOffsetPercent ||
        value.lfoOutOffsetPercent[i] > kPicoLfoMaxOffsetPercent) {
      return false;
    }
  }

  return value.lfoPulseWidth >= kPicoLfoMinPulseWidthPercent &&
         value.lfoPulseWidth <= kPicoLfoMaxPulseWidthPercent &&
         value.lfo2Link < kPicoLfoLinkCount &&
         value.lfo2PhaseOffset < kPicoLfoPhaseOffsetCount &&
         value.clockSource < kPicoClockSourceCount;
}

uint32_t settingsChecksumV11(const PicoSettingsV11& value) {
  return fnv1a(reinterpret_cast<const uint8_t*>(&value), offsetof(PicoSettingsV11, checksum));
}

bool settingsValidV11(const PicoSettingsV11& value) {
  return value.magic == SETTINGS_MAGIC &&
         value.version == SETTINGS_VERSION_PREVIOUS &&
         value.size == sizeof(PicoSettingsV11) &&
         value.checksum == settingsChecksumV11(value) &&
         value.heelRaw <= ADC_MAX &&
         value.toeRaw <= ADC_MAX &&
         value.rangeMv <= MAX_OCTAVE_SCALE_MV &&
         value.responseCents <= 6000 &&
         value.semitones >= kPicoMinSemitones &&
         value.semitones <= kPicoMaxSemitones;
}

PicoSettings makeDefaultPicoSettings() {
  PicoSettings value;
  memset(&value, 0, sizeof(value));
  value.heelRaw = 0;
  value.toeRaw = ADC_MAX;
  value.semitones = kPicoMaxSemitones;
  value.invert = 0;
  value.monitorEnabled = 1;
  value.bendDirection = static_cast<uint8_t>(kPicoBendUp);
  value.curveMode = kPicoCurveLinear;
  value.outputMode = kPicoOutputPedal;
  value.lfoWave = kPicoLfoSine;
  value.lfoLoRate = kPicoLfoLoRateSteps;
  value.lfoFmRate = kPicoLfoFmRateSteps;
  value.lfoDepthPercent = kPicoLfoMaxDepthPercent;
  value.outputFocus = OUTPUT_FOCUS_EXP;
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    value.lfoOutMode[i] = kPicoOutputLfoLo;
    value.lfoOutWave[i] = i == 0 ? kPicoLfoSine : kPicoLfoTriangle;
    value.lfoOutLoRate[i] = kPicoDefaultLfoLoRate;
    value.lfoOutFmRate[i] = kPicoDefaultLfoFmRate;
    value.lfoOutDepthPercent[i] = kPicoLfoMaxDepthPercent;
    value.lfoOutPolarity[i] = static_cast<uint8_t>(kPicoBendUp);
    value.lfoOutPulseWidth[i] = kPicoDefaultLfoPulseWidthPercent;
    value.lfoOutOffsetPercent[i] = 0;
  }
  value.lfoPulseWidth = kPicoDefaultLfoPulseWidthPercent;
  value.lfo2Link = kPicoLfoLinkOff;
  value.lfo2PhaseOffset = kPicoLfoPhase0;
  value.clockSource = kPicoClockOff;
  value.rangeMv = DEFAULT_RANGE_MV;
  value.responseCents = kDefaultFullScaleResponseCents;
  fillToeMapFromCurrentFit(value);
  finalizeSettings(value);
  return value;
}

void configureProcessor() {
  PedalCalibration calibration;
  calibration.heelRaw = settings.heelRaw;
  calibration.toeRaw = settings.toeRaw;
  calibration.invert = settings.invert != 0;
  calibration.snapLow = PEDAL_SNAP_LOW;
  calibration.snapHigh = PEDAL_SNAP_HIGH;
  calibration.deadband = PEDAL_DEADBAND;

  PedalFilterSettings filterSettings;
  filterSettings.enabled = true;
  filterSettings.minCutoffHz = 3.5f;
  filterSettings.beta = 0.12f;
  filterSettings.derivativeCutoffHz = 1.0f;

  pedalProcessor.configure(calibration, filterSettings, settings.curveMode);
  pedalProcessor.reset();
}

void markSettingsDirty() {
  settingsDirty = true;
  lastSettingsChangeMs = millis();
  finalizeSettings(settings);
}

void forceDisplayUpdate() {
  displayForceUpdate = true;
  lastRenderedCalState = -1;
  lastRenderedSemitones = 999;
  lastRenderedTuneMode = false;
  lastRenderedCvOverrideEnabled = false;
  lastRenderedCvOverrideMv = 0xffff;
  lastRenderedMenuActive = false;
  lastRenderedMenuIndex = 0xff;
  lastRenderedMenuEditField = 0xff;
  lastRenderedMenuEditValue = 0xff;
  lastRenderedCurveMode = 0xff;
  lastRenderedBendDirection = 0xff;
  lastRenderedOutputFocus = 0xff;
  lastRenderedOutputMode = 0xff;
  lastRenderedLfoWave = 0xff;
  lastRenderedLfoLoRate = 0xff;
  lastRenderedLfoFmRate = 0xff;
  lastRenderedLfoDepthPercent = 0xff;
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    lastRenderedLfoOutMode[i] = 0xff;
    lastRenderedLfoOutWave[i] = 0xff;
    lastRenderedLfoOutLoRate[i] = 0xff;
    lastRenderedLfoOutFmRate[i] = 0xff;
    lastRenderedLfoOutDepthPercent[i] = 0xff;
    lastRenderedLfoOutPolarity[i] = 0xff;
    lastRenderedLfoOutPulseWidth[i] = 0xff;
    lastRenderedLfoOutOffsetPercent[i] = -128;
  }
  lastRenderedLfoPulseWidth = 0xff;
  lastRenderedLfo2Link = 0xff;
  lastRenderedLfo2PhaseOffset = 0xff;
  lastRenderedClockSource = 0xff;
}

void saveSettings() {
  finalizeSettings(settings);
#if EXPCTRL_PICO_USE_EEPROM
  EEPROM.put(0, settings);
  EEPROM.commit();
  settingsDirty = false;
  Serial.println(F("OK saved settings to Pico flash"));
#else
  settingsDirty = false;
  Serial.println(F("OK settings kept in RAM; this board core has no EEPROM persistence enabled"));
#endif
}

void loadSettings() {
#if EXPCTRL_PICO_USE_EEPROM
  EEPROM.begin(sizeof(PicoSettings));
  PicoSettings stored;
  EEPROM.get(0, stored);
  if (settingsValid(stored)) {
    settings = stored;
    settingsDirty = false;
    return;
  }

  PicoSettingsV11 storedV11;
  EEPROM.get(0, storedV11);
  if (settingsValidV11(storedV11)) {
    settings = makeDefaultPicoSettings();
    settings.heelRaw = storedV11.heelRaw;
    settings.toeRaw = storedV11.toeRaw;
    settings.semitones = storedV11.semitones;
    settings.invert = storedV11.invert;
    settings.monitorEnabled = storedV11.monitorEnabled;
    settings.bendDirection =
        storedV11.bendDirection <= static_cast<uint8_t>(kPicoBendDown)
            ? storedV11.bendDirection
            : static_cast<uint8_t>(kPicoBendUp);
    settings.curveMode = clampPicoCurveMode(storedV11.curveMode);
    settings.outputMode = clampPicoOutputMode(storedV11.outputMode);
    settings.lfoWave = clampPicoLfoWave(storedV11.lfoWave);
    settings.lfoLoRate = clampPicoLfoRateStep(storedV11.lfoLoRate, kPicoOutputLfoLo);
    settings.lfoFmRate = clampPicoLfoRateStep(storedV11.lfoFmRate, kPicoOutputLfoFm);
    settings.lfoDepthPercent = clampPicoLfoDepthPercent(storedV11.lfoDepthPercent);
    settings.outputFocus =
        storedV11.outputFocus < OUTPUT_FOCUS_COUNT ? storedV11.outputFocus : OUTPUT_FOCUS_EXP;
    for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
      settings.lfoOutMode[i] = clampPicoLfoOutputMode(storedV11.lfoOutMode[i]);
      settings.lfoOutWave[i] = clampPicoLfoWave(storedV11.lfoOutWave[i]);
      settings.lfoOutLoRate[i] =
          clampPicoLfoRateStep(storedV11.lfoOutLoRate[i], kPicoOutputLfoLo);
      settings.lfoOutFmRate[i] =
          clampPicoLfoRateStep(storedV11.lfoOutFmRate[i], kPicoOutputLfoFm);
      settings.lfoOutDepthPercent[i] =
          clampPicoLfoDepthPercent(storedV11.lfoOutDepthPercent[i]);
      settings.lfoOutPolarity[i] = storedV11.lfoOutPolarity[i] > 1
                                       ? static_cast<uint8_t>(kPicoBendUp)
                                       : storedV11.lfoOutPolarity[i];
    }
    settings.rangeMv = storedV11.rangeMv;
    settings.responseCents = storedV11.responseCents;
    memcpy(settings.toeMapMv, storedV11.toeMapMv, sizeof(settings.toeMapMv));
    finalizeSettings(settings);
    settingsDirty = true;
    Serial.println(F("Settings migrated from version 11; calibration and toe map preserved"));
    return;
  }
#endif

  settings = makeDefaultPicoSettings();
  settingsDirty = true;
}

void setDisplayMessage(const char* top, const char* bottom, uint32_t durationMs) {
  strncpy(displayMessageTop, top, sizeof(displayMessageTop) - 1);
  displayMessageTop[sizeof(displayMessageTop) - 1] = '\0';
  strncpy(displayMessageBottom, bottom, sizeof(displayMessageBottom) - 1);
  displayMessageBottom[sizeof(displayMessageBottom) - 1] = '\0';
  displayMessageUntilMs = millis() + durationMs;
  forceDisplayUpdate();
}

void signedIntervalLabel(int semitones, char* out, size_t outSize) {
  int clamped = clampPicoSemitones(semitones);
  const char* absLabel = picoAbsoluteIntervalLabel(static_cast<uint8_t>(abs(clamped)));
  if (clamped > 0) {
    snprintf(out, outSize, "+%s", absLabel);
  } else if (clamped < 0) {
    snprintf(out, outSize, "-%s", absLabel);
  } else {
    snprintf(out, outSize, "1");
  }
}

uint16_t readPedalRaw() {
  uint32_t total = 0;
  for (uint8_t i = 0; i < PEDAL_OVERSAMPLES; ++i) {
    total += analogRead(PEDAL_PIN);
  }
  return static_cast<uint16_t>((total + (PEDAL_OVERSAMPLES / 2)) / PEDAL_OVERSAMPLES);
}

uint16_t readPedalEndpointRaw() {
  constexpr uint8_t endpointSamples = 16;
  uint32_t total = 0;
  for (uint8_t i = 0; i < endpointSamples; ++i) {
    total += readPedalRaw();
    delay(1);
  }
  return static_cast<uint16_t>((total + (endpointSamples / 2)) / endpointSamples);
}

PicoBendDirection currentBendDirection() {
  return settings.bendDirection == static_cast<uint8_t>(kPicoBendDown)
             ? kPicoBendDown
             : kPicoBendUp;
}

const char* directionLabel() {
  return currentBendDirection() == kPicoBendDown ? "DOWN" : "UP";
}

uint8_t currentOutputFocus() {
  return settings.outputFocus < OUTPUT_FOCUS_COUNT ? settings.outputFocus : OUTPUT_FOCUS_EXP;
}

bool outputFocusIsExp() {
  return currentOutputFocus() == OUTPUT_FOCUS_EXP;
}

bool outputFocusIsLfoOut() {
  return !outputFocusIsExp();
}

uint8_t currentLfoOutIndex() {
  return currentOutputFocus() == OUTPUT_FOCUS_LFO2 ? 1 : 0;
}

const char* outputFocusLabel(uint8_t focus) {
  switch (focus < OUTPUT_FOCUS_COUNT ? focus : OUTPUT_FOCUS_EXP) {
    case OUTPUT_FOCUS_LFO1:
      return "LFO1";
    case OUTPUT_FOCUS_LFO2:
      return "LFO2";
    case OUTPUT_FOCUS_EXP:
    default:
      return "EXP";
  }
}

const char* currentOutputFocusLabel() {
  return outputFocusLabel(currentOutputFocus());
}

const char* polarityLabel(uint8_t polarity) {
  return polarity == static_cast<uint8_t>(kPicoBendDown) ? "DN" : "UP";
}

bool outputModeIsLfo() {
  return settings.outputMode == kPicoOutputLfoLo || settings.outputMode == kPicoOutputLfoFm;
}

const char* outputModeLabel() {
  return picoOutputModeName(settings.outputMode);
}

const char* outputModeDisplayLabel() {
  return picoOutputModeDisplayLabel(settings.outputMode);
}

const char* lfoWaveLabel() {
  return picoLfoWaveName(settings.lfoWave);
}

const char* lfoWaveDisplayLabel() {
  return picoLfoWaveDisplayLabel(settings.lfoWave);
}

const char* curveLabel() {
  return picoCurveName(settings.curveMode);
}

const char* curveDisplayLabel() {
  return picoCurveDisplayLabel(settings.curveMode);
}

uint8_t currentIntervalMagnitude() {
  return picoIntervalMagnitude(settings.semitones);
}

int8_t signedIntervalForCurrentDirection(uint8_t magnitude) {
  return signedPicoInterval(magnitude, currentBendDirection());
}

int32_t noBendOutputMicrovolts() {
  return picoUnipolarNoBendMicrovolts(currentBendDirection());
}

uint8_t currentLfoRateStep() {
  return settings.outputMode == kPicoOutputLfoFm ? settings.lfoFmRate : settings.lfoLoRate;
}

float configuredLfoMaxRateHz() {
  return computePicoLfoRateHz(settings.outputMode, currentLfoRateStep());
}

float currentLfoRateHz() {
  return computePicoLfoRateHzForPedal(settings.outputMode, pedalState.filtered, currentLfoRateStep());
}

uint8_t lfoOutRateStep(uint8_t index) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  return settings.lfoOutMode[safeIndex] == kPicoOutputLfoFm ? settings.lfoOutFmRate[safeIndex]
                                                            : settings.lfoOutLoRate[safeIndex];
}

float lfoOutRateHz(uint8_t index) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  return computePicoLfoRateHz(settings.lfoOutMode[safeIndex], lfoOutRateStep(safeIndex));
}

float currentFocusedLfoRateHz() {
  if (outputFocusIsLfoOut()) {
    return lfoOutRateHz(currentLfoOutIndex());
  }
  return computePicoLfoRateHz(settings.outputMode, currentLfoRateStep());
}

uint8_t currentFocusedLfoDepthPercent() {
  if (outputFocusIsLfoOut()) {
    return settings.lfoOutDepthPercent[currentLfoOutIndex()];
  }
  return settings.lfoDepthPercent;
}

uint8_t currentFocusedLfoWave() {
  if (outputFocusIsLfoOut()) {
    return settings.lfoOutWave[currentLfoOutIndex()];
  }
  return settings.lfoWave;
}

uint8_t currentFocusedOutputMode() {
  if (outputFocusIsLfoOut()) {
    return settings.lfoOutMode[currentLfoOutIndex()];
  }
  return settings.outputMode;
}

const char* currentFocusedLfoWaveDisplayLabel() {
  return picoLfoWaveDisplayLabel(currentFocusedLfoWave());
}

void formatLfoRate(char* out, size_t outSize, float hz) {
  if (hz < 10.0f) {
    snprintf(out, outSize, "%.2fHZ", hz);
  } else if (hz < 100.0f) {
    snprintf(out, outSize, "%.1fHZ", hz);
  } else {
    snprintf(out, outSize, "%.0fHZ", hz);
  }
}

void formatLfoDepth(char* out, size_t outSize, uint8_t depthPercent) {
  snprintf(out, outSize, "%u%%", clampPicoLfoDepthPercent(depthPercent));
}

int32_t toeOutputMicrovoltsForSemitone(int semitones) {
  if (clampPicoSemitones(semitones) == 0) {
    return noBendOutputMicrovolts();
  }
  return static_cast<int32_t>(settings.toeMapMv[picoSemitoneMapIndex(semitones)]) * 1000L;
}

void updateOutputFromPedalState() {
  if (cvOverrideEnabled) {
    currentOutputMicrovolts = static_cast<int32_t>(cvOverrideMv) * 1000L;
  } else if (outputModeIsLfo()) {
    uint32_t nowUs = micros();
    float dt = lastLfoUs == 0 ? static_cast<float>(CONTROL_PERIOD_US) / 1000000.0f
                              : static_cast<float>(nowUs - lastLfoUs) / 1000000.0f;
    lastLfoUs = nowUs;
    dt = clampValue(dt, 0.0f, 0.050f);
    lfoPhase += currentLfoRateHz() * dt;
    float wrapped = floorf(lfoPhase);
    lfoCycle += static_cast<uint32_t>(wrapped);
    lfoPhase -= wrapped;

    float value = computePicoLfoWaveValue(lfoPhase,
                                          settings.lfoWave,
                                          lfoCycle,
                                          LFO_SEED_EXP,
                                          settings.lfoPulseWidth);
    if (currentBendDirection() == kPicoBendDown) {
      value = 1.0f - value;
    }
    value = attenuatePicoLfoWaveValue(value, settings.lfoDepthPercent);
    currentOutputMicrovolts = static_cast<int32_t>(
        lroundf(value * static_cast<float>(kPicoDacFullScaleMicrovolts)));
  } else {
    lastLfoUs = 0;
    currentOutputMicrovolts =
        computePicoMappedOutputMicrovolts(pedalState.curved,
                                          noBendOutputMicrovolts(),
                                          toeOutputMicrovoltsForSemitone(settings.semitones));
  }
  currentDacCode = microvoltsToMcp4728Code(currentOutputMicrovolts);
}

bool lfo2Linked() {
  return settings.lfo2Link != kPicoLfoLinkOff;
}

void updateLfoOutState(uint8_t index) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;

  if (safeIndex == 1 && lfo2Linked()) {
    // LFO2 rides LFO1's phase so the pair can never drift apart.
    computePicoLinkedPhase(lfoOutPhase[0],
                           lfoOutCycle[0],
                           settings.lfo2Link,
                           settings.lfo2PhaseOffset,
                           &lfoOutPhase[1],
                           &lfoOutCycle[1]);
    lastLfoOutUs[1] = 0;
  } else {
    uint32_t nowUs = micros();
    float dt = lastLfoOutUs[safeIndex] == 0
                   ? static_cast<float>(CONTROL_PERIOD_US) / 1000000.0f
                   : static_cast<float>(nowUs - lastLfoOutUs[safeIndex]) / 1000000.0f;
    lastLfoOutUs[safeIndex] = nowUs;
    dt = clampValue(dt, 0.0f, 0.050f);

    lfoOutPhase[safeIndex] += lfoOutRateHz(safeIndex) * dt;
    float wrapped = floorf(lfoOutPhase[safeIndex]);
    lfoOutCycle[safeIndex] += static_cast<uint32_t>(wrapped);
    lfoOutPhase[safeIndex] -= wrapped;
  }

  float value = computePicoLfoWaveValue(lfoOutPhase[safeIndex],
                                        settings.lfoOutWave[safeIndex],
                                        lfoOutCycle[safeIndex],
                                        LFO_SEED_OUT[safeIndex],
                                        settings.lfoOutPulseWidth[safeIndex]);
  if (settings.lfoOutPolarity[safeIndex] == static_cast<uint8_t>(kPicoBendDown)) {
    value = 1.0f - value;
  }
  value = attenuatePicoLfoWaveValue(value, settings.lfoOutDepthPercent[safeIndex]);
  value = offsetPicoLfoWaveValue(value, settings.lfoOutOffsetPercent[safeIndex]);
  lfoOutOutputMicrovolts[safeIndex] =
      static_cast<int32_t>(lroundf(value * static_cast<float>(kPicoDacFullScaleMicrovolts)));
  lfoOutDacCode[safeIndex] = microvoltsToMcp4728Code(lfoOutOutputMicrovolts[safeIndex]);
}

void updateClockState() {
  if (settings.clockSource == kPicoClockOff) {
    clockDacCode = 0;
    return;
  }
  uint8_t source = settings.clockSource == kPicoClockLfo2 ? 1 : 0;
  // Full-swing square at the source LFO's rate, ignoring its depth/offset.
  clockDacCode = lfoOutPhase[source] < 0.5f ? kMcp4728MaxCode : 0;
}

void updateLfoOutStates() {
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    updateLfoOutState(i);
  }
  updateClockState();
}

bool probeI2c(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

uint16_t dacChannelCode(uint8_t channel) {
  switch (channel) {
    case 0:
      return currentDacCode;
    case 1:
      return lfoOutDacCode[0];
    case 2:
      return lfoOutDacCode[1];
    case 3:
    default:
      return clockDacCode;
  }
}

// Multi-write (UDAC=0) rather than fast-write: it latches the outputs even if
// the LDAC pin floats high, and stamps VREF=VDD / gain=1 / power-on into every
// update so a previously reconfigured chip cannot skew the output scale.
bool writeMcp4728(uint16_t channelA, uint16_t channelB, uint16_t channelC, uint16_t channelD) {
  uint16_t codes[DAC_CHANNEL_COUNT] = {
      clampValue<uint16_t>(channelA, 0, kMcp4728MaxCode),
      clampValue<uint16_t>(channelB, 0, kMcp4728MaxCode),
      clampValue<uint16_t>(channelC, 0, kMcp4728MaxCode),
      clampValue<uint16_t>(channelD, 0, kMcp4728MaxCode),
  };

  Wire.beginTransmission(MCP4728_ADDRESS);
  for (uint8_t i = 0; i < DAC_CHANNEL_COUNT; ++i) {
    Wire.write(static_cast<uint8_t>(0x40 | (i << 1)));
    Wire.write(static_cast<uint8_t>((codes[i] >> 8) & 0x0F));
    Wire.write(static_cast<uint8_t>(codes[i] & 0xFF));
  }
  return Wire.endTransmission() == 0;
}

// Program 0V into every channel's power-on EEPROM so the jacks come up silent
// between power-on and firmware boot. Run on demand via "dac eeprom".
bool programMcp4728PowerOnDefaults() {
  Wire.beginTransmission(MCP4728_ADDRESS);
  Wire.write(0x50); // Sequential write with EEPROM, starting at channel A, UDAC=0.
  for (uint8_t i = 0; i < DAC_CHANNEL_COUNT; ++i) {
    Wire.write(0x00); // VREF=VDD, normal power, gain=1, code high nibble 0.
    Wire.write(0x00);
  }
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(60); // EEPROM write time.
  return true;
}

bool dacCodeChangeShouldWrite(uint8_t channel, uint16_t code) {
  if (channel >= DAC_CHANNEL_COUNT || lastWrittenDacCodes[channel] < 0) {
    return true;
  }
  if (code == 0 || code == kMcp4728MaxCode) {
    return static_cast<int>(code) != lastWrittenDacCodes[channel];
  }
  int delta = abs(static_cast<int>(code) - lastWrittenDacCodes[channel]);
  return delta >= static_cast<int>(DAC_CODE_DEADBAND);
}

void writeOutputToDac(bool force = false) {
  currentDacCode = microvoltsToMcp4728Code(currentOutputMicrovolts);
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    lfoOutDacCode[i] = microvoltsToMcp4728Code(lfoOutOutputMicrovolts[i]);
  }
  if (!dacReady) {
    return;
  }

  bool shouldWrite = force;
  for (uint8_t channel = 0; channel < DAC_CHANNEL_COUNT; ++channel) {
    if (dacCodeChangeShouldWrite(channel, dacChannelCode(channel))) {
      shouldWrite = true;
      break;
    }
  }
  if (!shouldWrite) {
    return;
  }

  uint32_t nowUs = micros();
  uint32_t minIntervalUs = DAC_LFO_WRITE_MIN_INTERVAL_US;
  if (!force &&
      static_cast<uint32_t>(nowUs - lastDacWriteUs) < minIntervalUs) {
    return;
  }

  if (writeMcp4728(currentDacCode, lfoOutDacCode[0], lfoOutDacCode[1], clockDacCode)) {
    for (uint8_t channel = 0; channel < DAC_CHANNEL_COUNT; ++channel) {
      lastWrittenDacCodes[channel] = dacChannelCode(channel);
    }
    lastDacWriteUs = nowUs;
  } else {
    dacReady = false;
    Serial.println(F("MCP4728 write failed; outputs paused until dac probe passes"));
  }
}

void refreshControlNow() {
  currentRaw = readPedalRaw();
  pedalState = pedalProcessor.process(currentRaw, 1000.0f);
  updateOutputFromPedalState();
  updateLfoOutStates();
  if (!sleeping) {
    writeOutputToDac(true);
  }
}

void ssd1306Command(uint8_t command) {
  Wire.beginTransmission(SSD1306_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();
}

void ssd1306Command2(uint8_t command, uint8_t value) {
  Wire.beginTransmission(SSD1306_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.write(value);
  Wire.endTransmission();
}

bool initDisplay() {
  if (!probeI2c(SSD1306_ADDRESS)) {
    return false;
  }

  ssd1306Command(0xAE);
  ssd1306Command2(0xD5, 0x80);
  ssd1306Command2(0xA8, 0x1F);
  ssd1306Command2(0xD3, 0x00);
  ssd1306Command(0x40);
  ssd1306Command2(0x8D, 0x14);
  ssd1306Command2(0x20, 0x00);
  ssd1306Command(0xA1);
  ssd1306Command(0xC8);
  ssd1306Command2(0xDA, 0x02);
  ssd1306Command2(0x81, 0x8F);
  ssd1306Command2(0xD9, 0xF1);
  ssd1306Command2(0xDB, 0x40);
  ssd1306Command(0xA4);
  ssd1306Command(0xA6);
  ssd1306Command(0xAF);
  return true;
}

void displayOn() {
  if (displayReady) {
    ssd1306Command(0xAF);
  }
}

void displayOff() {
  if (displayReady) {
    ssd1306Command(0xAE);
  }
}

void clearDisplayBuffer() {
  memset(displayBuffer, 0, sizeof(displayBuffer));
}

void setPixel(int x, int y, bool on = true) {
  if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) {
    return;
  }

  uint16_t index = static_cast<uint16_t>(x + (y / 8) * DISPLAY_WIDTH);
  uint8_t mask = static_cast<uint8_t>(1u << (y & 7));
  if (on) {
    displayBuffer[index] |= mask;
  } else {
    displayBuffer[index] &= ~mask;
  }
}

void fillRect(int x, int y, int w, int h, bool on = true) {
  for (int yy = y; yy < y + h; ++yy) {
    for (int xx = x; xx < x + w; ++xx) {
      setPixel(xx, yy, on);
    }
  }
}

#define GLYPH3X5(r0, r1, r2, r3, r4) \
  static_cast<uint16_t>((((r0) & 7) << 12) | (((r1) & 7) << 9) | (((r2) & 7) << 6) | (((r3) & 7) << 3) | ((r4) & 7))

uint16_t glyph3x5(char c) {
  if (c >= 'a' && c <= 'z' && c != 'b') {
    c = static_cast<char>(c - 'a' + 'A');
  }

  switch (c) {
    case '0':
    case 'O':
      return GLYPH3X5(7, 5, 5, 5, 7);
    case '1':
      return GLYPH3X5(2, 6, 2, 2, 7);
    case '2':
      return GLYPH3X5(7, 1, 7, 4, 7);
    case '3':
      return GLYPH3X5(7, 1, 7, 1, 7);
    case '4':
      return GLYPH3X5(5, 5, 7, 1, 1);
    case '5':
    case 'S':
      return GLYPH3X5(7, 4, 7, 1, 7);
    case '6':
      return GLYPH3X5(7, 4, 7, 5, 7);
    case '7':
      return GLYPH3X5(7, 1, 1, 1, 1);
    case '8':
      return GLYPH3X5(7, 5, 7, 5, 7);
    case '9':
      return GLYPH3X5(7, 5, 7, 1, 7);
    case 'A':
      return GLYPH3X5(7, 5, 7, 5, 5);
    case 'b':
    case 'B':
      return GLYPH3X5(4, 4, 6, 5, 6);
    case 'C':
      return GLYPH3X5(7, 4, 4, 4, 7);
    case 'D':
      return GLYPH3X5(6, 5, 5, 5, 6);
    case 'E':
      return GLYPH3X5(7, 4, 6, 4, 7);
    case 'F':
      return GLYPH3X5(7, 4, 6, 4, 4);
    case 'G':
      return GLYPH3X5(7, 4, 5, 5, 7);
    case 'H':
      return GLYPH3X5(5, 5, 7, 5, 5);
    case 'I':
      return GLYPH3X5(7, 2, 2, 2, 7);
    case 'K':
      return GLYPH3X5(5, 5, 6, 5, 5);
    case 'L':
      return GLYPH3X5(4, 4, 4, 4, 7);
    case 'M':
      return GLYPH3X5(5, 7, 7, 5, 5);
    case 'N':
      return GLYPH3X5(5, 7, 7, 7, 5);
    case 'P':
      return GLYPH3X5(7, 5, 7, 4, 4);
    case 'Q':
      return GLYPH3X5(7, 5, 5, 7, 1);
    case 'R':
      return GLYPH3X5(7, 5, 6, 5, 5);
    case 'T':
      return GLYPH3X5(7, 2, 2, 2, 2);
    case 'U':
      return GLYPH3X5(5, 5, 5, 5, 7);
    case 'V':
      return GLYPH3X5(5, 5, 5, 5, 2);
    case 'W':
      return GLYPH3X5(5, 5, 7, 7, 5);
    case 'X':
      return GLYPH3X5(5, 5, 2, 5, 5);
    case 'Y':
      return GLYPH3X5(5, 5, 2, 2, 2);
    case 'Z':
      return GLYPH3X5(7, 1, 2, 4, 7);
    case '+':
      return GLYPH3X5(0, 2, 7, 2, 0);
    case '-':
      return GLYPH3X5(0, 0, 7, 0, 0);
    case '.':
      return GLYPH3X5(0, 0, 0, 0, 2);
    case ':':
      return GLYPH3X5(0, 2, 0, 2, 0);
    case '>':
      return GLYPH3X5(4, 2, 1, 2, 4);
    case '%':
      return GLYPH3X5(5, 1, 2, 4, 5);
    case ' ':
    default:
      return 0;
  }
}

void drawGlyph3x5(int x, int y, char c, uint8_t scale) {
  uint16_t bits = glyph3x5(c);
  for (int row = 0; row < 5; ++row) {
    uint8_t rowMask = static_cast<uint8_t>((bits >> ((4 - row) * 3)) & 7);
    for (int col = 0; col < 3; ++col) {
      if (rowMask & (1 << (2 - col))) {
        fillRect(x + col * scale, y + row * scale, scale, scale, true);
      }
    }
  }
}

int drawText3x5(int x, int y, const char* text, uint8_t scale) {
  int cursor = x;
  while (*text != '\0') {
    drawGlyph3x5(cursor, y, *text, scale);
    cursor += 4 * scale;
    ++text;
  }
  return cursor;
}

void sendDisplayBuffer() {
  if (!displayReady) {
    return;
  }

  ssd1306Command(0x21);
  ssd1306Command(0);
  ssd1306Command(DISPLAY_WIDTH - 1);
  ssd1306Command(0x22);
  ssd1306Command(0);
  ssd1306Command(3);

  for (uint16_t offset = 0; offset < DISPLAY_BUFFER_SIZE; offset += 16) {
    Wire.beginTransmission(SSD1306_ADDRESS);
    Wire.write(0x40);
    for (uint8_t i = 0; i < 16; ++i) {
      Wire.write(displayBuffer[offset + i]);
    }
    Wire.endTransmission();
  }
}

bool menuItemVisible(uint8_t item) {
  bool focusedLfoSettingsApply = outputFocusIsLfoOut() || outputModeIsLfo();
  if ((item == MENU_WAVE || item == MENU_DEPTH) && !focusedLfoSettingsApply) {
    return false;
  }
  if (item == MENU_PW &&
      (!focusedLfoSettingsApply || currentFocusedLfoWave() != kPicoLfoPulse)) {
    return false;
  }
  if (item == MENU_OFS && !outputFocusIsLfoOut()) {
    return false;
  }
  if ((item == MENU_LINK || item == MENU_PHS) &&
      currentOutputFocus() != OUTPUT_FOCUS_LFO2) {
    return false;
  }
  if (item == MENU_PHS && !lfo2Linked()) {
    return false;
  }
  if ((item == MENU_CURVE || item == MENU_CAL) && outputFocusIsLfoOut()) {
    return false;
  }
  return item < MENU_COUNT;
}

uint8_t visibleMenuItemCount() {
  uint8_t count = 0;
  for (uint8_t item = 0; item < MENU_COUNT; ++item) {
    if (menuItemVisible(item)) {
      ++count;
    }
  }
  return count;
}

uint8_t visibleMenuItemPosition(uint8_t selectedItem) {
  uint8_t position = 0;
  for (uint8_t item = 0; item < MENU_COUNT; ++item) {
    if (menuItemVisible(item)) {
      ++position;
    }
    if (item == selectedItem) {
      return position;
    }
  }
  return 1;
}

uint8_t lfoModeEditValueToMode(uint8_t value) {
  return value == 0 ? kPicoOutputLfoLo : kPicoOutputLfoFm;
}

uint8_t lfoModeEditValueFromMode(uint8_t mode) {
  return clampPicoLfoOutputMode(mode) == kPicoOutputLfoFm ? 1 : 0;
}

const char* lfoModeEditValueLabel(uint8_t value) {
  return picoOutputModeDisplayLabel(lfoModeEditValueToMode(value));
}

const char* menuItemTitle(uint8_t item) {
  switch (item) {
    case MENU_OUT:
      return "OUT";
    case MENU_MODE:
      return "MODE";
    case MENU_WAVE:
      return "WAVE";
    case MENU_DEPTH:
      return "DEPTH";
    case MENU_PW:
      return "PW";
    case MENU_OFS:
      return "OFS";
    case MENU_LINK:
      return "LINK";
    case MENU_PHS:
      return "PHS";
    case MENU_CLK:
      return "CLK";
    case MENU_CURVE:
      return "CURVE";
    case MENU_CAL:
      return "CAL";
    case MENU_DIR:
      return outputFocusIsLfoOut() || outputModeIsLfo() ? "POL" : "DIR";
    case MENU_DONE:
      return "DONE";
    default:
      return "MENU";
  }
}

uint8_t currentFocusedLfoPulseWidth() {
  if (outputFocusIsLfoOut()) {
    return settings.lfoOutPulseWidth[currentLfoOutIndex()];
  }
  return settings.lfoPulseWidth;
}

const char* lfoLinkDisplayLabel(uint8_t link) {
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
      return "OFF";
  }
}

const char* lfoPhaseOffsetDisplayLabel(uint8_t phaseOffset) {
  switch (clampPicoLfoPhaseOffset(phaseOffset)) {
    case kPicoLfoPhase90:
      return "90";
    case kPicoLfoPhase180:
      return "180";
    case kPicoLfoPhase270:
      return "270";
    case kPicoLfoPhase0:
    default:
      return "0";
  }
}

const char* menuItemValue(uint8_t item) {
  switch (item) {
    case MENU_OUT:
      return currentOutputFocusLabel();
    case MENU_MODE:
      return outputFocusIsLfoOut()
                 ? picoOutputModeDisplayLabel(settings.lfoOutMode[currentLfoOutIndex()])
                 : outputModeDisplayLabel();
    case MENU_WAVE:
      return outputFocusIsLfoOut()
                 ? picoLfoWaveDisplayLabel(settings.lfoOutWave[currentLfoOutIndex()])
                 : lfoWaveDisplayLabel();
    case MENU_DEPTH: {
      static char depth[8];
      formatLfoDepth(depth, sizeof(depth), currentFocusedLfoDepthPercent());
      return depth;
    }
    case MENU_PW: {
      static char pw[8];
      snprintf(pw, sizeof(pw), "%u%%", currentFocusedLfoPulseWidth());
      return pw;
    }
    case MENU_OFS: {
      static char ofs[8];
      snprintf(ofs, sizeof(ofs), "%+d%%",
               static_cast<int>(settings.lfoOutOffsetPercent[currentLfoOutIndex()]));
      return ofs;
    }
    case MENU_LINK:
      return lfoLinkDisplayLabel(settings.lfo2Link);
    case MENU_PHS:
      return lfoPhaseOffsetDisplayLabel(settings.lfo2PhaseOffset);
    case MENU_CLK:
      return picoClockSourceDisplayLabel(settings.clockSource);
    case MENU_CURVE:
      return curveDisplayLabel();
    case MENU_CAL:
      return "PEDAL";
    case MENU_DIR:
      return outputFocusIsLfoOut()
                 ? polarityLabel(settings.lfoOutPolarity[currentLfoOutIndex()])
                 : directionLabel();
    case MENU_DONE:
      return "OK";
    default:
      return "";
  }
}

const char* menuEditTitle(uint8_t field) {
  switch (field) {
    case MENU_EDIT_OUT:
      return "SET OUT";
    case MENU_EDIT_MODE:
      return "SET MODE";
    case MENU_EDIT_WAVE:
      return "SET WAVE";
    case MENU_EDIT_DEPTH:
      return "SET DEP";
    case MENU_EDIT_PW:
      return "SET PW";
    case MENU_EDIT_OFS:
      return "SET OFS";
    case MENU_EDIT_LINK:
      return "SET LINK";
    case MENU_EDIT_PHS:
      return "SET PHS";
    case MENU_EDIT_CLK:
      return "SET CLK";
    case MENU_EDIT_CURVE:
      return "SET CURVE";
    default:
      return "SET";
  }
}

uint8_t menuEditValueCount(uint8_t field) {
  switch (field) {
    case MENU_EDIT_OUT:
      return OUTPUT_FOCUS_COUNT;
    case MENU_EDIT_MODE:
      return outputFocusIsLfoOut() ? 2 : kPicoOutputModeCount;
    case MENU_EDIT_WAVE:
      return kPicoLfoWaveCount;
    case MENU_EDIT_DEPTH:
      return kPicoLfoDepthStepCount;
    case MENU_EDIT_PW:
      return kPicoLfoPulseWidthStepCount;
    case MENU_EDIT_OFS:
      return kPicoLfoOffsetStepCount;
    case MENU_EDIT_LINK:
      return kPicoLfoLinkCount;
    case MENU_EDIT_PHS:
      return kPicoLfoPhaseOffsetCount;
    case MENU_EDIT_CLK:
      return kPicoClockSourceCount;
    case MENU_EDIT_CURVE:
      return kPicoCurveCount;
    default:
      return 1;
  }
}

uint8_t currentMenuEditValue(uint8_t field) {
  switch (field) {
    case MENU_EDIT_OUT:
      return currentOutputFocus();
    case MENU_EDIT_MODE:
      return outputFocusIsLfoOut()
                 ? lfoModeEditValueFromMode(settings.lfoOutMode[currentLfoOutIndex()])
                 : settings.outputMode;
    case MENU_EDIT_WAVE:
      return outputFocusIsLfoOut() ? settings.lfoOutWave[currentLfoOutIndex()]
                                   : settings.lfoWave;
    case MENU_EDIT_DEPTH:
      return picoLfoDepthIndexFromPercent(currentFocusedLfoDepthPercent());
    case MENU_EDIT_PW:
      return picoLfoPulseWidthIndexFromPercent(currentFocusedLfoPulseWidth());
    case MENU_EDIT_OFS:
      return picoLfoOffsetIndexFromPercent(
          settings.lfoOutOffsetPercent[currentLfoOutIndex()]);
    case MENU_EDIT_LINK:
      return settings.lfo2Link;
    case MENU_EDIT_PHS:
      return settings.lfo2PhaseOffset;
    case MENU_EDIT_CLK:
      return settings.clockSource;
    case MENU_EDIT_CURVE:
      return settings.curveMode;
    default:
      return 0;
  }
}

const char* menuEditValueLabel(uint8_t field, uint8_t value) {
  switch (field) {
    case MENU_EDIT_OUT:
      return outputFocusLabel(value);
    case MENU_EDIT_MODE:
      return outputFocusIsLfoOut() ? lfoModeEditValueLabel(value)
                                   : picoOutputModeDisplayLabel(value);
    case MENU_EDIT_WAVE:
      return picoLfoWaveDisplayLabel(value);
    case MENU_EDIT_DEPTH: {
      static char depth[8];
      formatLfoDepth(depth, sizeof(depth), picoLfoDepthPercentFromIndex(value));
      return depth;
    }
    case MENU_EDIT_PW: {
      static char pw[8];
      snprintf(pw, sizeof(pw), "%u%%", picoLfoPulseWidthPercentFromIndex(value));
      return pw;
    }
    case MENU_EDIT_OFS: {
      static char ofs[8];
      snprintf(ofs, sizeof(ofs), "%+d%%",
               static_cast<int>(picoLfoOffsetPercentFromIndex(value)));
      return ofs;
    }
    case MENU_EDIT_LINK:
      return lfoLinkDisplayLabel(value);
    case MENU_EDIT_PHS:
      return lfoPhaseOffsetDisplayLabel(value);
    case MENU_EDIT_CLK:
      return picoClockSourceDisplayLabel(value);
    case MENU_EDIT_CURVE:
      return picoCurveDisplayLabel(value);
    default:
      return "";
  }
}

bool lfoOutRenderStateUnchanged() {
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    if (lastRenderedLfoOutMode[i] != settings.lfoOutMode[i] ||
        lastRenderedLfoOutWave[i] != settings.lfoOutWave[i] ||
        lastRenderedLfoOutLoRate[i] != settings.lfoOutLoRate[i] ||
        lastRenderedLfoOutFmRate[i] != settings.lfoOutFmRate[i] ||
        lastRenderedLfoOutDepthPercent[i] != settings.lfoOutDepthPercent[i] ||
        lastRenderedLfoOutPolarity[i] != settings.lfoOutPolarity[i] ||
        lastRenderedLfoOutPulseWidth[i] != settings.lfoOutPulseWidth[i] ||
        lastRenderedLfoOutOffsetPercent[i] != settings.lfoOutOffsetPercent[i]) {
      return false;
    }
  }
  return lastRenderedLfoPulseWidth == settings.lfoPulseWidth &&
         lastRenderedLfo2Link == settings.lfo2Link &&
         lastRenderedLfo2PhaseOffset == settings.lfo2PhaseOffset &&
         lastRenderedClockSource == settings.clockSource;
}

void rememberLfoOutRenderState() {
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    lastRenderedLfoOutMode[i] = settings.lfoOutMode[i];
    lastRenderedLfoOutWave[i] = settings.lfoOutWave[i];
    lastRenderedLfoOutLoRate[i] = settings.lfoOutLoRate[i];
    lastRenderedLfoOutFmRate[i] = settings.lfoOutFmRate[i];
    lastRenderedLfoOutDepthPercent[i] = settings.lfoOutDepthPercent[i];
    lastRenderedLfoOutPolarity[i] = settings.lfoOutPolarity[i];
    lastRenderedLfoOutPulseWidth[i] = settings.lfoOutPulseWidth[i];
    lastRenderedLfoOutOffsetPercent[i] = settings.lfoOutOffsetPercent[i];
  }
  lastRenderedLfoPulseWidth = settings.lfoPulseWidth;
  lastRenderedLfo2Link = settings.lfo2Link;
  lastRenderedLfo2PhaseOffset = settings.lfo2PhaseOffset;
  lastRenderedClockSource = settings.clockSource;
}

void renderDisplay() {
  if (!displayReady || sleeping) {
    return;
  }

  bool messageActive =
      displayMessageUntilMs != 0 && static_cast<int32_t>(millis() - displayMessageUntilMs) < 0;
  if (!messageActive && displayMessageUntilMs != 0) {
    displayMessageUntilMs = 0;
    forceDisplayUpdate();
  }

  if (calState != CAL_IDLE) {
    if (!displayForceUpdate && !lastRenderedMessage &&
        lastRenderedCalState == static_cast<int>(calState)) {
      return;
    }

    clearDisplayBuffer();
    drawText3x5(0, 0, calState == CAL_WAIT_HEEL ? "HEEL" : "TOE", 3);
    drawText3x5(0, 24, "PRESS ENCODER", 1);
    sendDisplayBuffer();
    displayForceUpdate = false;
    lastRenderedMessage = false;
    lastRenderedCalState = static_cast<int>(calState);
    return;
  }

  if (menuActive) {
    if (!displayForceUpdate &&
        lastRenderedMenuActive &&
        lastRenderedMenuIndex == menuIndex &&
        lastRenderedMenuEditField == menuEditField &&
        lastRenderedMenuEditValue == menuEditValue &&
        lastRenderedCurveMode == settings.curveMode &&
        lastRenderedBendDirection == settings.bendDirection &&
        lastRenderedOutputFocus == currentOutputFocus() &&
        lastRenderedOutputMode == settings.outputMode &&
        lastRenderedLfoWave == settings.lfoWave &&
        lastRenderedLfoLoRate == settings.lfoLoRate &&
        lastRenderedLfoFmRate == settings.lfoFmRate &&
        lastRenderedLfoDepthPercent == settings.lfoDepthPercent &&
        lfoOutRenderStateUnchanged() &&
        lastRenderedSettingsDirty == settingsDirty) {
      return;
    }

    clearDisplayBuffer();
    if (menuEditField != MENU_EDIT_NONE) {
      drawText3x5(0, 0, menuEditTitle(menuEditField), 1);
      drawText3x5(0, 10, menuEditValueLabel(menuEditField, menuEditValue), 3);
      drawText3x5(0, 27, "TURN PRESS OK", 1);
    } else {
      char line[24];
      snprintf(line,
               sizeof(line),
               "MENU %u/%u",
               static_cast<unsigned>(visibleMenuItemPosition(menuIndex)),
               static_cast<unsigned>(visibleMenuItemCount()));
      drawText3x5(0, 0, line, 1);
      drawText3x5(0, 10, menuItemTitle(menuIndex), 2);
      drawText3x5(70, 10, menuItemValue(menuIndex), 2);
      drawText3x5(0, 27, "TURN ITEM PRESS OK", 1);
    }
    sendDisplayBuffer();
    displayForceUpdate = false;
    lastRenderedMessage = false;
    lastRenderedMenuActive = true;
    lastRenderedMenuIndex = menuIndex;
    lastRenderedMenuEditField = menuEditField;
    lastRenderedMenuEditValue = menuEditValue;
    lastRenderedCurveMode = settings.curveMode;
    lastRenderedBendDirection = settings.bendDirection;
    lastRenderedOutputFocus = currentOutputFocus();
    lastRenderedOutputMode = settings.outputMode;
    lastRenderedLfoWave = settings.lfoWave;
    lastRenderedLfoLoRate = settings.lfoLoRate;
    lastRenderedLfoFmRate = settings.lfoFmRate;
    lastRenderedLfoDepthPercent = settings.lfoDepthPercent;
    rememberLfoOutRenderState();
    lastRenderedSettingsDirty = settingsDirty;
    return;
  }

  if (messageActive) {
    if (!displayForceUpdate && lastRenderedMessage) {
      return;
    }
    clearDisplayBuffer();
    drawText3x5(0, 0, displayMessageTop, 3);
    drawText3x5(0, 22, displayMessageBottom, 1);
    sendDisplayBuffer();
    displayForceUpdate = false;
    lastRenderedMessage = true;
    return;
  }

  char label[8];
  if (outputFocusIsLfoOut()) {
    snprintf(label, sizeof(label), "%s", currentOutputFocusLabel());
  } else if (cvOverrideEnabled) {
    snprintf(label, sizeof(label), "CV");
  } else if (outputModeIsLfo()) {
    snprintf(label, sizeof(label), "%s", outputModeDisplayLabel());
  } else {
    signedIntervalLabel(settings.semitones, label, sizeof(label));
  }
  // Keep the normal OLED page static. Live pedal/output values remain in
  // Serial Monitor, where ADC jitter cannot cause visible OLED redraw flicker.
  if (!displayForceUpdate &&
      !lastRenderedMessage &&
      lastRenderedSemitones == settings.semitones &&
      lastRenderedDacReady == dacReady &&
      lastRenderedSettingsDirty == settingsDirty &&
      lastRenderedTuneMode == tuneMode &&
      lastRenderedCvOverrideEnabled == cvOverrideEnabled &&
      lastRenderedCvOverrideMv == cvOverrideMv &&
      lastRenderedCurveMode == settings.curveMode &&
      lastRenderedBendDirection == settings.bendDirection &&
      lastRenderedOutputFocus == currentOutputFocus() &&
      lastRenderedOutputMode == settings.outputMode &&
      lastRenderedLfoWave == settings.lfoWave &&
      lastRenderedLfoLoRate == settings.lfoLoRate &&
      lastRenderedLfoFmRate == settings.lfoFmRate &&
      lastRenderedLfoDepthPercent == settings.lfoDepthPercent &&
      lfoOutRenderStateUnchanged()) {
    return;
  }

  clearDisplayBuffer();
  drawText3x5(0, 0, label, 4);

  char line[24];
  if (outputFocusIsLfoOut()) {
    if (currentLfoOutIndex() == 1 && lfo2Linked()) {
      snprintf(line, sizeof(line), "LNK %s %s",
               lfoLinkDisplayLabel(settings.lfo2Link),
               lfoPhaseOffsetDisplayLabel(settings.lfo2PhaseOffset));
    } else {
      char rate[16];
      formatLfoRate(rate, sizeof(rate), lfoOutRateHz(currentLfoOutIndex()));
      snprintf(line, sizeof(line), "SPD %s", rate);
    }
  } else if (cvOverrideEnabled) {
    snprintf(line, sizeof(line), "SET %uMV", cvOverrideMv);
  } else if (outputModeIsLfo()) {
    snprintf(line, sizeof(line), "SPD PED");
  } else {
    int32_t toeUv = toeOutputMicrovoltsForSemitone(settings.semitones);
    snprintf(line, sizeof(line), "TOE %ldMV", static_cast<long>(toeUv / 1000));
  }
  drawText3x5(70, 1, line, 1);

  if (outputFocusIsLfoOut()) {
    int8_t ofs = settings.lfoOutOffsetPercent[currentLfoOutIndex()];
    if (ofs != 0) {
      snprintf(line, sizeof(line), "D%u O%+d", currentFocusedLfoDepthPercent(), ofs);
    } else {
      snprintf(line, sizeof(line), "DEP %u%%", currentFocusedLfoDepthPercent());
    }
  } else if (cvOverrideEnabled) {
    snprintf(line, sizeof(line), "FIXED");
  } else if (outputModeIsLfo()) {
    snprintf(line, sizeof(line), "DEP %u%%", settings.lfoDepthPercent);
  } else if (tuneMode) {
    snprintf(line, sizeof(line), "STEP %uMV", tuneStepMv);
  } else if (settings.responseCents > 0) {
    snprintf(line, sizeof(line), "RSP %uC", settings.responseCents);
  } else {
    snprintf(line, sizeof(line), "RNG %uMV", settings.rangeMv);
  }
  drawText3x5(70, 10, line, 1);

  if (outputFocusIsLfoOut()) {
    snprintf(line,
             sizeof(line),
             "%s %s",
             currentFocusedLfoWaveDisplayLabel(),
             polarityLabel(settings.lfoOutPolarity[currentLfoOutIndex()]));
  } else if (outputModeIsLfo()) {
    snprintf(line,
             sizeof(line),
             "%s %s",
             lfoWaveDisplayLabel(),
             currentBendDirection() == kPicoBendDown ? "DN" : "UP");
  } else {
    snprintf(line, sizeof(line), "CRV %s", curveDisplayLabel());
  }
  drawText3x5(70, 19, line, 1);

  if (!dacReady) {
    drawText3x5(0, 27, "DAC FAIL", 1);
  } else if (cvOverrideEnabled) {
    drawText3x5(0, 27, "CV HOLD", 1);
  } else if (tuneMode) {
    drawText3x5(0, 27, "TUNE", 1);
  } else if (outputFocusIsLfoOut()) {
    snprintf(line, sizeof(line), "%s %s", currentOutputFocusLabel(), settingsDirty ? "DIRTY" : "SAVED");
    drawText3x5(0, 27, line, 1);
  } else if (outputModeIsLfo()) {
    snprintf(line, sizeof(line), "LFO %s", settingsDirty ? "DIRTY" : "SAVED");
    drawText3x5(0, 27, line, 1);
  } else {
    snprintf(line,
             sizeof(line),
             "%s %s",
             currentBendDirection() == kPicoBendDown ? "DN" : "UP",
             settingsDirty ? "DIRTY" : "SAVED");
    drawText3x5(0, 27, line, 1);
  }

  sendDisplayBuffer();
  displayForceUpdate = false;
  lastRenderedMessage = false;
  lastRenderedMenuActive = false;
  lastRenderedCalState = static_cast<int>(calState);
  lastRenderedSemitones = settings.semitones;
  lastRenderedDacReady = dacReady;
  lastRenderedSettingsDirty = settingsDirty;
  lastRenderedTuneMode = tuneMode;
  lastRenderedCvOverrideEnabled = cvOverrideEnabled;
  lastRenderedCvOverrideMv = cvOverrideMv;
  lastRenderedCurveMode = settings.curveMode;
  lastRenderedBendDirection = settings.bendDirection;
  lastRenderedOutputFocus = currentOutputFocus();
  lastRenderedOutputMode = settings.outputMode;
  lastRenderedLfoWave = settings.lfoWave;
  lastRenderedLfoLoRate = settings.lfoLoRate;
  lastRenderedLfoFmRate = settings.lfoFmRate;
  lastRenderedLfoDepthPercent = settings.lfoDepthPercent;
  rememberLfoOutRenderState();
}

void renderDisplayNow() {
  if (!displayReady) {
    return;
  }
  displayOn();
  forceDisplayUpdate();
  lastDisplayMs = millis();
  renderDisplay();
}

void printHelp();

void printStatus() {
  char label[8];
  signedIntervalLabel(settings.semitones, label, sizeof(label));

  Serial.print(F("raw="));
  Serial.print(pedalState.raw);
  Serial.print(F(" norm="));
  Serial.print(pedalState.normalized, 4);
  Serial.print(F(" filt="));
  Serial.print(pedalState.filtered, 4);
  Serial.print(F(" interval="));
  Serial.print(label);
  Serial.print(F(" semis="));
  Serial.print(settings.semitones);
  Serial.print(F(" focus="));
  Serial.print(currentOutputFocusLabel());
  Serial.print(F(" expMv="));
  Serial.print(currentOutputMicrovolts / 1000.0f, 3);
  Serial.print(F(" expCode="));
  Serial.print(currentDacCode);
  Serial.print(F(" mode="));
  Serial.print(outputModeLabel());
  Serial.print(F(" dir="));
  Serial.print(directionLabel());
  if (outputModeIsLfo()) {
    Serial.print(F(" wave="));
    Serial.print(lfoWaveLabel());
    Serial.print(F(" rateHz="));
    Serial.print(currentLfoRateHz(), 3);
    Serial.print(F(" maxRateHz="));
    Serial.print(configuredLfoMaxRateHz(), 3);
    Serial.print(F(" depth="));
    Serial.print(settings.lfoDepthPercent);
    Serial.print(F("%"));
    Serial.print(F(" phase="));
    Serial.print(lfoPhase, 4);
  }
  Serial.print(F(" noBendMv="));
  Serial.print(noBendOutputMicrovolts() / 1000L);
  Serial.print(F(" rangeMv="));
  Serial.print(settings.rangeMv);
  Serial.print(F(" responseCents="));
  Serial.print(settings.responseCents);
  Serial.print(F(" curve="));
  Serial.print(curveLabel());
  Serial.print(F(" toeMapMv="));
  Serial.print(toeOutputMicrovoltsForSemitone(settings.semitones) / 1000L);
  Serial.print(F(" tune="));
  Serial.print(tuneMode ? F("on") : F("off"));
  Serial.print(F(" tuneStepMv="));
  Serial.print(tuneStepMv);
  Serial.print(F(" cv="));
  if (cvOverrideEnabled) {
    Serial.print(cvOverrideMv);
    Serial.print(F("mV"));
  } else {
    Serial.print(F("off"));
  }
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    Serial.print(F(" lfo"));
    Serial.print(i + 1);
    Serial.print(F("Mv="));
    Serial.print(lfoOutOutputMicrovolts[i] / 1000.0f, 3);
    Serial.print(F(" code="));
    Serial.print(lfoOutDacCode[i]);
    Serial.print(F(" mode="));
    Serial.print(picoOutputModeName(settings.lfoOutMode[i]));
    Serial.print(F(" wave="));
    Serial.print(picoLfoWaveName(settings.lfoOutWave[i]));
    Serial.print(F(" rateHz="));
    Serial.print(lfoOutRateHz(i), 3);
    Serial.print(F(" depth="));
    Serial.print(settings.lfoOutDepthPercent[i]);
    Serial.print(F("%"));
    Serial.print(F(" pol="));
    Serial.print(polarityLabel(settings.lfoOutPolarity[i]));
    Serial.print(F(" pw="));
    Serial.print(settings.lfoOutPulseWidth[i]);
    Serial.print(F("%"));
    Serial.print(F(" ofs="));
    Serial.print(settings.lfoOutOffsetPercent[i]);
    Serial.print(F("%"));
  }
  Serial.print(F(" link="));
  Serial.print(picoLfoLinkName(settings.lfo2Link));
  Serial.print(F(" linkPhase="));
  Serial.print(picoLfoPhaseOffsetDegrees(settings.lfo2PhaseOffset));
  Serial.print(F(" clock="));
  Serial.print(picoClockSourceName(settings.clockSource));
  Serial.print(F(" clockCode="));
  Serial.print(clockDacCode);
  Serial.print(F(" dac="));
  Serial.print(dacReady ? F("ready") : F("missing"));
  Serial.print(F(" oled="));
  Serial.print(displayReady ? F("ready") : F("missing"));
  Serial.print(F(" cal="));
  Serial.print(calState == CAL_WAIT_HEEL ? F("heel") : (calState == CAL_WAIT_TOE ? F("toe") : F("idle")));
  Serial.print(F(" sleep="));
  Serial.print(sleeping ? F("on") : F("off"));
  Serial.print(F(" encBtn13="));
  Serial.print(digitalRead(ENCODER_SWITCH_PIN) == LOW ? F("PRESSED") : F("open"));
  Serial.print(F(" encA14="));
  Serial.print(digitalRead(ENCODER_A_PIN) == LOW ? F("LOW") : F("HIGH"));
  Serial.print(F(" encB15="));
  Serial.print(digitalRead(ENCODER_B_PIN) == LOW ? F("LOW") : F("HIGH"));
  Serial.print(F(" heel="));
  Serial.print(settings.heelRaw);
  Serial.print(F(" toe="));
  Serial.print(settings.toeRaw);
  Serial.print(F(" invert="));
  Serial.println(settings.invert ? F("on") : F("off"));
}

void setIntervalMagnitude(int magnitude, bool persist) {
  uint8_t clampedMagnitude =
      static_cast<uint8_t>(clampValue<int>(magnitude, 0, kPicoMaxSemitones));
  settings.semitones = signedIntervalForCurrentDirection(clampedMagnitude);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();

  char label[8];
  signedIntervalLabel(settings.semitones, label, sizeof(label));
  Serial.print(F("OK interval "));
  Serial.println(label);
}

void setSemitones(int semitones, bool persist) {
  int clampedSemitones = clampPicoSemitones(semitones);
  if (clampedSemitones > 0) {
    settings.bendDirection = static_cast<uint8_t>(kPicoBendUp);
  } else if (clampedSemitones < 0) {
    settings.bendDirection = static_cast<uint8_t>(kPicoBendDown);
  }
  setIntervalMagnitude(picoIntervalMagnitude(clampedSemitones), persist);
}

void setBendDirection(PicoBendDirection direction, bool persist) {
  settings.bendDirection = static_cast<uint8_t>(direction);
  settings.semitones = signedIntervalForCurrentDirection(currentIntervalMagnitude());
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK direction "));
  Serial.println(directionLabel());
}

void setCurveMode(uint8_t mode, bool persist) {
  settings.curveMode = clampPicoCurveMode(mode);
  configureProcessor();
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK curve "));
  Serial.println(curveLabel());
}

void resetLfoPhase() {
  lfoPhase = 0.0f;
  lfoCycle = 0;
  lastLfoUs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.println(F("OK LFO phase reset"));
}

void resetLfoOutPhase(uint8_t index) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  lfoOutPhase[safeIndex] = 0.0f;
  lfoOutCycle[safeIndex] = 0;
  lastLfoOutUs[safeIndex] = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK "));
  Serial.print(outputFocusLabel(static_cast<uint8_t>(OUTPUT_FOCUS_LFO1 + safeIndex)));
  Serial.println(F(" phase reset"));
}

void resetFocusedLfoPhase() {
  if (outputFocusIsLfoOut()) {
    resetLfoOutPhase(currentLfoOutIndex());
  } else {
    resetLfoPhase();
  }
}

void setOutputFocus(uint8_t focus, bool persist) {
  settings.outputFocus = static_cast<uint8_t>(
      clampValue<int>(focus, OUTPUT_FOCUS_EXP, OUTPUT_FOCUS_COUNT - 1));
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  forceDisplayUpdate();
  renderDisplayNow();
  Serial.print(F("OK focus "));
  Serial.println(currentOutputFocusLabel());
}

void cycleOutputFocus() {
  setOutputFocus(static_cast<uint8_t>((currentOutputFocus() + 1) % OUTPUT_FOCUS_COUNT), true);
  saveSettings();
  setDisplayMessage(currentOutputFocusLabel(), "EDIT TARGET", CAL_MESSAGE_MS);
  renderDisplayNow();
}

void setOutputMode(uint8_t mode, bool persist) {
  settings.outputMode = clampPicoOutputMode(mode);
  tuneMode = false;
  lfoPhase = 0.0f;
  lfoCycle = 0;
  lastLfoUs = 0;
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK mode "));
  Serial.println(outputModeLabel());
}

void setLfoOutMode(uint8_t index, uint8_t mode, bool persist) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  settings.lfoOutMode[safeIndex] = clampPicoLfoOutputMode(mode);
  lfoOutPhase[safeIndex] = 0.0f;
  lfoOutCycle[safeIndex] = 0;
  lastLfoOutUs[safeIndex] = 0;
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK "));
  Serial.print(outputFocusLabel(static_cast<uint8_t>(OUTPUT_FOCUS_LFO1 + safeIndex)));
  Serial.print(F(" mode "));
  Serial.println(picoOutputModeName(settings.lfoOutMode[safeIndex]));
}

void setFocusedOutputMode(uint8_t mode, bool persist) {
  if (outputFocusIsLfoOut()) {
    setLfoOutMode(currentLfoOutIndex(), mode, persist);
  } else {
    setOutputMode(mode, persist);
  }
}

void setLfoWave(uint8_t wave, bool persist) {
  settings.lfoWave = clampPicoLfoWave(wave);
  lfoPhase = 0.0f;
  lfoCycle = 0;
  lastLfoUs = 0;
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK wave "));
  Serial.println(lfoWaveLabel());
}

void setLfoOutWave(uint8_t index, uint8_t wave, bool persist) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  settings.lfoOutWave[safeIndex] = clampPicoLfoWave(wave);
  lfoOutPhase[safeIndex] = 0.0f;
  lfoOutCycle[safeIndex] = 0;
  lastLfoOutUs[safeIndex] = 0;
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK "));
  Serial.print(outputFocusLabel(static_cast<uint8_t>(OUTPUT_FOCUS_LFO1 + safeIndex)));
  Serial.print(F(" wave "));
  Serial.println(picoLfoWaveName(settings.lfoOutWave[safeIndex]));
}

void setFocusedLfoWave(uint8_t wave, bool persist) {
  if (outputFocusIsLfoOut()) {
    setLfoOutWave(currentLfoOutIndex(), wave, persist);
  } else {
    setLfoWave(wave, persist);
  }
}

void setCurrentLfoRateStep(int step, bool persist) {
  if (settings.outputMode == kPicoOutputLfoFm) {
    settings.lfoFmRate = clampPicoLfoRateStep(step, settings.outputMode);
  } else {
    settings.lfoLoRate = clampPicoLfoRateStep(step, settings.outputMode);
  }
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();

  char rate[16];
  formatLfoRate(rate, sizeof(rate), configuredLfoMaxRateHz());
  Serial.print(F("OK max rate "));
  Serial.print(rate);
  Serial.print(F(" step="));
  Serial.println(currentLfoRateStep());
}

void setCurrentLfoRateHz(float hz, bool persist) {
  setCurrentLfoRateStep(nearestPicoLfoRateStep(settings.outputMode, hz), persist);
}

void setLfoOutRateStep(uint8_t index, int step, bool persist) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  if (settings.lfoOutMode[safeIndex] == kPicoOutputLfoFm) {
    settings.lfoOutFmRate[safeIndex] =
        clampPicoLfoRateStep(step, settings.lfoOutMode[safeIndex]);
  } else {
    settings.lfoOutLoRate[safeIndex] =
        clampPicoLfoRateStep(step, settings.lfoOutMode[safeIndex]);
  }
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();

  char rate[16];
  formatLfoRate(rate, sizeof(rate), lfoOutRateHz(safeIndex));
  Serial.print(F("OK "));
  Serial.print(outputFocusLabel(static_cast<uint8_t>(OUTPUT_FOCUS_LFO1 + safeIndex)));
  Serial.print(F(" rate "));
  Serial.print(rate);
  Serial.print(F(" step="));
  Serial.println(lfoOutRateStep(safeIndex));
}

void setFocusedLfoRateStep(int step, bool persist) {
  if (outputFocusIsLfoOut()) {
    setLfoOutRateStep(currentLfoOutIndex(), step, persist);
  } else {
    setCurrentLfoRateStep(step, persist);
  }
}

void setFocusedLfoRateHz(float hz, bool persist) {
  if (outputFocusIsLfoOut()) {
    uint8_t index = currentLfoOutIndex();
    setLfoOutRateStep(index, nearestPicoLfoRateStep(settings.lfoOutMode[index], hz), persist);
  } else {
    setCurrentLfoRateHz(hz, persist);
  }
}

void setLfoDepthPercent(int value, bool persist) {
  settings.lfoDepthPercent = clampPicoLfoDepthPercent(value);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK depth "));
  Serial.print(settings.lfoDepthPercent);
  Serial.println(F("%"));
}

void setLfoOutDepthPercent(uint8_t index, int value, bool persist) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  settings.lfoOutDepthPercent[safeIndex] = clampPicoLfoDepthPercent(value);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK "));
  Serial.print(outputFocusLabel(static_cast<uint8_t>(OUTPUT_FOCUS_LFO1 + safeIndex)));
  Serial.print(F(" depth "));
  Serial.print(settings.lfoOutDepthPercent[safeIndex]);
  Serial.println(F("%"));
}

void setFocusedLfoDepthPercent(int value, bool persist) {
  if (outputFocusIsLfoOut()) {
    setLfoOutDepthPercent(currentLfoOutIndex(), value, persist);
  } else {
    setLfoDepthPercent(value, persist);
  }
}

void setFocusedLfoPulseWidth(int value, bool persist) {
  uint8_t clamped = clampPicoLfoPulseWidth(value);
  if (outputFocusIsLfoOut()) {
    settings.lfoOutPulseWidth[currentLfoOutIndex()] = clamped;
  } else {
    settings.lfoPulseWidth = clamped;
  }
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK "));
  Serial.print(currentOutputFocusLabel());
  Serial.print(F(" pw "));
  Serial.print(clamped);
  Serial.println(F("%"));
}

void setLfoOutOffsetPercent(uint8_t index, int value, bool persist) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  settings.lfoOutOffsetPercent[safeIndex] = clampPicoLfoOffsetPercent(value);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK "));
  Serial.print(outputFocusLabel(static_cast<uint8_t>(OUTPUT_FOCUS_LFO1 + safeIndex)));
  Serial.print(F(" offset "));
  if (settings.lfoOutOffsetPercent[safeIndex] >= 0) {
    Serial.print(F("+"));
  }
  Serial.print(settings.lfoOutOffsetPercent[safeIndex]);
  Serial.println(F("%"));
}

void setLfo2Link(uint8_t link, bool persist) {
  settings.lfo2Link = clampPicoLfoLink(link);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK LFO2 link "));
  Serial.println(picoLfoLinkName(settings.lfo2Link));
}

void setLfo2PhaseOffset(uint8_t phaseOffset, bool persist) {
  settings.lfo2PhaseOffset = clampPicoLfoPhaseOffset(phaseOffset);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK LFO2 link phase "));
  Serial.println(picoLfoPhaseOffsetDegrees(settings.lfo2PhaseOffset));
}

void setClockSource(uint8_t source, bool persist) {
  settings.clockSource = clampPicoClockSource(source);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK clock "));
  Serial.println(picoClockSourceName(settings.clockSource));
}

void resetAllLfoPhases() {
  lfoPhase = 0.0f;
  lfoCycle = 0;
  lastLfoUs = 0;
  for (uint8_t i = 0; i < LFO_OUT_COUNT; ++i) {
    lfoOutPhase[i] = 0.0f;
    lfoOutCycle[i] = 0;
    lastLfoOutUs[i] = 0;
  }
  refreshControlNow();
  renderDisplayNow();
  Serial.println(F("OK all LFO phases reset"));
}

bool parseOutputMode(const char* value, uint8_t* out) {
  if (value == nullptr || out == nullptr) {
    return false;
  }
  if (strcmp(value, "ped") == 0 || strcmp(value, "pedal") == 0 ||
      strcmp(value, "expr") == 0) {
    *out = kPicoOutputPedal;
    return true;
  }
  if (strcmp(value, "lo") == 0 || strcmp(value, "lfo") == 0 ||
      strcmp(value, "lfolo") == 0) {
    *out = kPicoOutputLfoLo;
    return true;
  }
  if (strcmp(value, "fm") == 0 || strcmp(value, "fast") == 0 ||
      strcmp(value, "audio") == 0 || strcmp(value, "lfofm") == 0) {
    *out = kPicoOutputLfoFm;
    return true;
  }
  return false;
}

bool parseOutputFocus(const char* value, uint8_t* out) {
  if (value == nullptr || out == nullptr) {
    return false;
  }
  if (strcmp(value, "exp") == 0 || strcmp(value, "expr") == 0 ||
      strcmp(value, "ped") == 0 || strcmp(value, "pedal") == 0) {
    *out = OUTPUT_FOCUS_EXP;
    return true;
  }
  if (strcmp(value, "lfo1") == 0 || strcmp(value, "l1") == 0 ||
      strcmp(value, "aux1") == 0) {
    *out = OUTPUT_FOCUS_LFO1;
    return true;
  }
  if (strcmp(value, "lfo2") == 0 || strcmp(value, "l2") == 0 ||
      strcmp(value, "aux2") == 0) {
    *out = OUTPUT_FOCUS_LFO2;
    return true;
  }
  return false;
}

bool parseLfoWave(const char* value, uint8_t* out) {
  if (value == nullptr || out == nullptr) {
    return false;
  }
  if (strcmp(value, "sine") == 0 || strcmp(value, "sin") == 0) {
    *out = kPicoLfoSine;
    return true;
  }
  if (strcmp(value, "triangle") == 0 || strcmp(value, "tri") == 0) {
    *out = kPicoLfoTriangle;
    return true;
  }
  if (strcmp(value, "sawup") == 0 || strcmp(value, "up") == 0 ||
      strcmp(value, "saw") == 0) {
    *out = kPicoLfoSawUp;
    return true;
  }
  if (strcmp(value, "sawdown") == 0 || strcmp(value, "sawdn") == 0 ||
      strcmp(value, "down") == 0 || strcmp(value, "dn") == 0) {
    *out = kPicoLfoSawDown;
    return true;
  }
  if (strcmp(value, "square") == 0 || strcmp(value, "sqr") == 0 ||
      strcmp(value, "sq") == 0) {
    *out = kPicoLfoSquare;
    return true;
  }
  if (strcmp(value, "pulse") == 0 || strcmp(value, "puls") == 0 ||
      strcmp(value, "pul") == 0) {
    *out = kPicoLfoPulse;
    return true;
  }
  if (strcmp(value, "sh") == 0 || strcmp(value, "samplehold") == 0 ||
      strcmp(value, "random") == 0 || strcmp(value, "rand") == 0) {
    *out = kPicoLfoSampleHold;
    return true;
  }
  if (strcmp(value, "drift") == 0 || strcmp(value, "wander") == 0 ||
      strcmp(value, "smoothrandom") == 0) {
    *out = kPicoLfoDrift;
    return true;
  }
  return false;
}

bool parseCurveMode(const char* value, uint8_t* out) {
  if (value == nullptr || out == nullptr) {
    return false;
  }
  if (strcmp(value, "linear") == 0 || strcmp(value, "lin") == 0) {
    *out = kPicoCurveLinear;
    return true;
  }
  if (strcmp(value, "easeout") == 0 || strcmp(value, "ease") == 0 ||
      strcmp(value, "sqrt") == 0 || strcmp(value, "eout") == 0) {
    *out = kPicoCurveEaseOut;
    return true;
  }
  if (strcmp(value, "square") == 0 || strcmp(value, "sqr") == 0) {
    *out = kPicoCurveSquare;
    return true;
  }
  if (strcmp(value, "smooth") == 0 || strcmp(value, "smth") == 0 ||
      strcmp(value, "smoothstep") == 0) {
    *out = kPicoCurveSmooth;
    return true;
  }
  return false;
}

void toggleBendDirection() {
  setBendDirection(currentBendDirection() == kPicoBendDown ? kPicoBendUp : kPicoBendDown, true);
  saveSettings();
  setDisplayMessage(directionLabel(), outputModeIsLfo() ? "POLARITY" : "DIRECTION", CAL_MESSAGE_MS);
  renderDisplayNow();
}

void setLfoOutPolarity(uint8_t index, uint8_t polarity, bool persist) {
  uint8_t safeIndex = index < LFO_OUT_COUNT ? index : 0;
  settings.lfoOutPolarity[safeIndex] =
      polarity == static_cast<uint8_t>(kPicoBendDown) ? static_cast<uint8_t>(kPicoBendDown)
                                                      : static_cast<uint8_t>(kPicoBendUp);
  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK "));
  Serial.print(outputFocusLabel(static_cast<uint8_t>(OUTPUT_FOCUS_LFO1 + safeIndex)));
  Serial.print(F(" polarity "));
  Serial.println(polarityLabel(settings.lfoOutPolarity[safeIndex]));
}

void toggleFocusedPolarityOrDirection() {
  if (outputFocusIsLfoOut()) {
    uint8_t index = currentLfoOutIndex();
    setLfoOutPolarity(index,
                      settings.lfoOutPolarity[index] == static_cast<uint8_t>(kPicoBendDown)
                          ? static_cast<uint8_t>(kPicoBendUp)
                          : static_cast<uint8_t>(kPicoBendDown),
                      true);
    saveSettings();
    return;
  }

  setBendDirection(currentBendDirection() == kPicoBendDown ? kPicoBendUp : kPicoBendDown,
                   true);
  saveSettings();
}

void setOctaveMillivolts(int value) {
  settings.rangeMv = static_cast<uint16_t>(clampValue<int>(value, 0, MAX_OCTAVE_SCALE_MV));
  settings.responseCents = 0;
  fillLinearToeMap(settings);
  markSettingsDirty();
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK rangeMv "));
  Serial.println(settings.rangeMv);
}

void resetToeMapToLinear() {
  fillToeMapFromCurrentFit(settings);
  markSettingsDirty();
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  if (settings.responseCents > 0) {
    Serial.print(F("OK map reset from responseCents "));
    Serial.println(settings.responseCents);
  } else {
    Serial.println(F("OK map reset from direction range"));
  }
}

void setResponseCents(int value) {
  if (value <= 0) {
    settings.responseCents = 0;
    fillLinearToeMap(settings);
    markSettingsDirty();
    displayMessageUntilMs = 0;
    refreshControlNow();
    renderDisplayNow();
    Serial.println(F("OK response off; linear voltage map restored"));
    return;
  }

  settings.responseCents = static_cast<uint16_t>(clampValue<int>(value, 100, 6000));
  fillResponseToeMap(settings);
  markSettingsDirty();
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK responseCents "));
  Serial.print(settings.responseCents);
  Serial.println(F("; map rebuilt"));
}

void setToeMapMillivolts(int semitones, int value, bool persist) {
  int clampedSemitones = clampPicoSemitones(semitones);
  uint16_t clampedMv = static_cast<uint16_t>(clampValue<int>(value, 0, MAX_OUTPUT_MV));

  if (clampedSemitones == 0) {
    Serial.println(F("ERR map 0 uses no-bend rail; set interval +/-1..9 first"));
    return;
  }
  settings.toeMapMv[picoSemitoneMapIndex(clampedSemitones)] = clampedMv;

  if (persist) {
    markSettingsDirty();
  }
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();

  char label[8];
  signedIntervalLabel(clampedSemitones, label, sizeof(label));
  Serial.print(F("OK map "));
  Serial.print(label);
  Serial.print(F(" toeMv="));
  Serial.println(settings.toeMapMv[picoSemitoneMapIndex(clampedSemitones)]);
}

void nudgeToeMapMillivolts(int deltaMv) {
  int semitones = clampPicoSemitones(settings.semitones);
  uint16_t currentMv = settings.toeMapMv[picoSemitoneMapIndex(semitones)];
  setToeMapMillivolts(semitones, static_cast<int>(currentMv) + deltaMv, true);
}

void printToeMap() {
  Serial.println(F("Pitch map toe voltages:"));
  for (int semitone = kPicoMinSemitones; semitone <= kPicoMaxSemitones; ++semitone) {
    char label[8];
    signedIntervalLabel(semitone, label, sizeof(label));
    Serial.print(F("  map "));
    Serial.print(semitone);
    Serial.print(F(" "));
    Serial.print(settings.toeMapMv[picoSemitoneMapIndex(semitone)]);
    Serial.print(F("   ; "));
    Serial.println(label);
  }
}

void setTuneMode(bool enabled) {
  if (enabled && outputModeIsLfo()) {
    Serial.println(F("ERR tune only applies in mode ped"));
    return;
  }
  tuneMode = enabled;
  if (tuneMode && cvOverrideEnabled) {
    cvOverrideEnabled = false;
  }
  sleeping = false;
  lastActivityMs = millis();
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.println(tuneMode ? F("OK tune on; turn encoder to adjust current interval toe mV, short-press to save/exit")
                          : F("OK tune off"));
}

void setTuneStepMillivolts(int value) {
  tuneStepMv = static_cast<uint16_t>(clampValue<int>(value, 1, MAX_TUNE_STEP_MV));
  forceDisplayUpdate();
  Serial.print(F("OK tune stepMv "));
  Serial.println(tuneStepMv);
}

void setCvOverrideMillivolts(int value) {
  cvOverrideMv = static_cast<uint16_t>(clampValue<int>(value, 0, MAX_OUTPUT_MV));
  cvOverrideEnabled = true;
  tuneMode = false;
  sleeping = false;
  lastActivityMs = millis();
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.print(F("OK cv fixed "));
  Serial.print(cvOverrideMv);
  Serial.println(F("mV; send cv off to resume pedal control"));
}

void clearCvOverride() {
  cvOverrideEnabled = false;
  sleeping = false;
  lastActivityMs = millis();
  displayMessageUntilMs = 0;
  refreshControlNow();
  renderDisplayNow();
  Serial.println(F("OK cv off; pedal control resumed"));
}

void enterSleep(const char* reason) {
  if (sleeping) {
    return;
  }

  sleeping = true;
  displayOff();
  Serial.print(F("SLEEP "));
  Serial.println(reason);
}

void wakeFromSleep(const char* reason) {
  if (!sleeping) {
    return;
  }

  sleeping = false;
  lastActivityMs = millis();
  displayOn();
  displayMessageUntilMs = 0;
  forceDisplayUpdate();
  Serial.print(F("WAKE "));
  Serial.println(reason);
}

void startCalibration() {
  pendingShortPress = false;
  sleeping = false;
  lastActivityMs = millis();
  displayMessageUntilMs = 0;
  calState = CAL_WAIT_HEEL;
  renderDisplayNow();
  Serial.println(F("CAL: put pedal heel-down, then short-press encoder"));
}

void captureHeel() {
  currentRaw = readPedalEndpointRaw();
  settings.heelRaw = currentRaw;
  calState = CAL_WAIT_TOE;
  configureProcessor();
  refreshControlNow();
  markSettingsDirty();
  lastActivityMs = millis();
  displayMessageUntilMs = 0;
  renderDisplayNow();
  Serial.print(F("CAL heel captured raw="));
  Serial.println(settings.heelRaw);
  Serial.println(F("CAL: put pedal toe-down, then short-press encoder"));
}

void captureToe() {
  currentRaw = readPedalEndpointRaw();
  uint16_t capturedToe = currentRaw;
  uint16_t span = settings.heelRaw > capturedToe
                      ? static_cast<uint16_t>(settings.heelRaw - capturedToe)
                      : static_cast<uint16_t>(capturedToe - settings.heelRaw);

  if (span < MIN_CAL_SPAN_RAW) {
    calState = CAL_WAIT_TOE;
    lastActivityMs = millis();
    displayMessageUntilMs = 0;
    renderDisplayNow();
    Serial.print(F("CAL toe rejected raw="));
    Serial.print(capturedToe);
    Serial.print(F(" span="));
    Serial.print(span);
    Serial.println(F(" too small; move pedal fully toe-down and press again"));
    return;
  }

  settings.toeRaw = capturedToe;
  calState = CAL_IDLE;
  configureProcessor();
  refreshControlNow();
  markSettingsDirty();
  saveSettings();
  lastActivityMs = millis();
  setDisplayMessage("SAVED", "CAL COMPLETE", 2500UL);
  renderDisplayNow();
  Serial.print(F("CAL toe captured raw="));
  Serial.println(settings.toeRaw);
  Serial.println(F("CAL: complete"));
}

void resetCalibration() {
  settings.heelRaw = 0;
  settings.toeRaw = ADC_MAX;
  settings.invert = 0;
  calState = CAL_IDLE;
  configureProcessor();
  refreshControlNow();
  markSettingsDirty();
  saveSettings();
  setDisplayMessage("RESET", "FULL ADC", CAL_MESSAGE_MS);
  renderDisplayNow();
}

void openMenu() {
  pendingShortPress = false;
  menuActive = true;
  menuIndex = MENU_OUT;
  menuEditField = MENU_EDIT_NONE;
  displayMessageUntilMs = 0;
  forceDisplayUpdate();
  renderDisplayNow();
  Serial.println(F("OK menu open; turn to item, press to edit/choose, hold to exit"));
}

void closeMenu() {
  pendingShortPress = false;
  menuActive = false;
  menuEditField = MENU_EDIT_NONE;
  displayMessageUntilMs = 0;
  forceDisplayUpdate();
  renderDisplayNow();
  Serial.println(F("OK menu closed"));
}

uint8_t wrapMenuChoice(int value, uint8_t count) {
  if (count == 0) {
    return 0;
  }
  while (value < 0) {
    value += count;
  }
  return static_cast<uint8_t>(value % count);
}

void beginMenuEdit(uint8_t field) {
  menuEditField = field;
  menuEditValue = currentMenuEditValue(field);
  forceDisplayUpdate();
  renderDisplayNow();
  Serial.print(F("OK edit "));
  Serial.print(menuEditTitle(menuEditField));
  Serial.print(F(" "));
  Serial.println(menuEditValueLabel(menuEditField, menuEditValue));
}

void moveMenuEdit(int direction) {
  menuEditValue =
      wrapMenuChoice(static_cast<int>(menuEditValue) + direction, menuEditValueCount(menuEditField));
  forceDisplayUpdate();
  renderDisplayNow();
  Serial.print(F("OK edit "));
  Serial.print(menuEditTitle(menuEditField));
  Serial.print(F(" "));
  Serial.println(menuEditValueLabel(menuEditField, menuEditValue));
}

void commitMenuEdit() {
  uint8_t field = menuEditField;
  uint8_t value = menuEditValue;
  switch (field) {
    case MENU_EDIT_OUT:
      setOutputFocus(value, true);
      saveSettings();
      break;
    case MENU_EDIT_MODE:
      setFocusedOutputMode(outputFocusIsLfoOut() ? lfoModeEditValueToMode(value) : value, true);
      saveSettings();
      break;
    case MENU_EDIT_WAVE:
      setFocusedLfoWave(value, true);
      saveSettings();
      break;
    case MENU_EDIT_DEPTH:
      setFocusedLfoDepthPercent(picoLfoDepthPercentFromIndex(value), true);
      saveSettings();
      break;
    case MENU_EDIT_PW:
      setFocusedLfoPulseWidth(picoLfoPulseWidthPercentFromIndex(value), true);
      saveSettings();
      break;
    case MENU_EDIT_OFS:
      setLfoOutOffsetPercent(currentLfoOutIndex(), picoLfoOffsetPercentFromIndex(value), true);
      saveSettings();
      break;
    case MENU_EDIT_LINK:
      setLfo2Link(value, true);
      saveSettings();
      break;
    case MENU_EDIT_PHS:
      setLfo2PhaseOffset(value, true);
      saveSettings();
      break;
    case MENU_EDIT_CLK:
      setClockSource(value, true);
      saveSettings();
      break;
    case MENU_EDIT_CURVE:
      setCurveMode(value, true);
      saveSettings();
      break;
    default:
      break;
  }
  menuEditField = MENU_EDIT_NONE;
  forceDisplayUpdate();
  renderDisplayNow();
}

void moveMenu(int direction) {
  if (menuEditField != MENU_EDIT_NONE) {
    moveMenuEdit(direction);
    return;
  }
  for (uint8_t attempts = 0; attempts < MENU_COUNT; ++attempts) {
    menuIndex = wrapMenuChoice(static_cast<int>(menuIndex) + direction, MENU_COUNT);
    if (menuItemVisible(menuIndex)) {
      break;
    }
  }
  forceDisplayUpdate();
  renderDisplayNow();
  Serial.print(F("OK menu "));
  Serial.println(menuItemTitle(menuIndex));
}

void selectMenuItem() {
  if (menuEditField != MENU_EDIT_NONE) {
    commitMenuEdit();
    return;
  }

  switch (menuIndex) {
    case MENU_OUT:
      beginMenuEdit(MENU_EDIT_OUT);
      break;
    case MENU_MODE:
      beginMenuEdit(MENU_EDIT_MODE);
      break;
    case MENU_WAVE:
      beginMenuEdit(MENU_EDIT_WAVE);
      break;
    case MENU_DEPTH:
      beginMenuEdit(MENU_EDIT_DEPTH);
      break;
    case MENU_PW:
      beginMenuEdit(MENU_EDIT_PW);
      break;
    case MENU_OFS:
      beginMenuEdit(MENU_EDIT_OFS);
      break;
    case MENU_LINK:
      beginMenuEdit(MENU_EDIT_LINK);
      break;
    case MENU_PHS:
      beginMenuEdit(MENU_EDIT_PHS);
      break;
    case MENU_CLK:
      beginMenuEdit(MENU_EDIT_CLK);
      break;
    case MENU_CURVE:
      beginMenuEdit(MENU_EDIT_CURVE);
      break;
    case MENU_CAL:
      menuActive = false;
      menuEditField = MENU_EDIT_NONE;
      forceDisplayUpdate();
      startCalibration();
      break;
    case MENU_DIR:
      toggleFocusedPolarityOrDirection();
      forceDisplayUpdate();
      renderDisplayNow();
      break;
    case MENU_DONE:
    default:
      closeMenu();
      break;
  }
}

// Tap tempo: two single presses on a focused LFO both sync the phase and, when
// the gap lands inside the tap window, set the rate to the tapped interval.
bool applyTapTempo() {
  uint32_t now = millis();
  uint32_t delta = now - lastTapMs;
  bool tapped = lastTapMs != 0 && lastTapFocus == currentOutputFocus() &&
                delta >= TAP_MIN_MS && delta <= TAP_MAX_MS;
  lastTapMs = now;
  lastTapFocus = currentOutputFocus();
  if (!tapped) {
    return false;
  }
  if (currentOutputFocus() == OUTPUT_FOCUS_LFO2 && lfo2Linked()) {
    Serial.println(F("LFO2 linked to LFO1; tap ignored until link off"));
    return false;
  }
  setFocusedLfoRateHz(1000.0f / static_cast<float>(delta), true);
  return true;
}

void performSingleShortPress() {
  if (outputFocusIsLfoOut()) {
    bool tapped = applyTapTempo();
    resetLfoOutPhase(currentLfoOutIndex());
    setDisplayMessage(tapped ? "TAP" : "SYNC", currentOutputFocusLabel(), CAL_MESSAGE_MS);
    renderDisplayNow();
    return;
  }
  if (outputModeIsLfo()) {
    bool tapped = applyTapTempo();
    resetLfoPhase();
    setDisplayMessage(tapped ? "TAP" : "SYNC", "LFO PHASE", CAL_MESSAGE_MS);
    renderDisplayNow();
    return;
  }
  setIntervalMagnitude(0, true);
  saveSettings();
  setDisplayMessage("ZERO", "INTERVAL 1", CAL_MESSAGE_MS);
  renderDisplayNow();
}

void handleShortPress() {
  if (menuActive) {
    pendingShortPress = false;
    selectMenuItem();
    return;
  }

  if (calState == CAL_WAIT_HEEL) {
    pendingShortPress = false;
    captureHeel();
    return;
  }
  if (calState == CAL_WAIT_TOE) {
    pendingShortPress = false;
    captureToe();
    return;
  }

  if (tuneMode) {
    pendingShortPress = false;
    tuneMode = false;
    saveSettings();
    setDisplayMessage("SAVED", "TUNE OFF", CAL_MESSAGE_MS);
    renderDisplayNow();
    Serial.println(F("OK tune off saved"));
    return;
  }

  uint32_t now = millis();
  if (pendingShortPress && now - pendingShortPressMs <= BUTTON_DOUBLE_CLICK_MS) {
    pendingShortPress = false;
    cycleOutputFocus();
    return;
  }

  pendingShortPress = true;
  pendingShortPressMs = now;
}

void pollPendingShortPress() {
  if (!pendingShortPress) {
    return;
  }
  if (stableButtonPressed) {
    return;
  }
  if (millis() - pendingShortPressMs < BUTTON_DOUBLE_CLICK_MS) {
    return;
  }
  pendingShortPress = false;
  performSingleShortPress();
}

uint8_t readEncoderState() {
  uint8_t a = digitalRead(ENCODER_A_PIN) == LOW ? 1 : 0;
  uint8_t b = digitalRead(ENCODER_B_PIN) == LOW ? 1 : 0;
  return static_cast<uint8_t>((a << 1) | b);
}

void handleEncoderInterrupt() {
  uint8_t next = readEncoderState();
  if (next == lastEncoderState) {
    return;
  }

  uint8_t transition = static_cast<uint8_t>((lastEncoderState << 2) | next);
  lastEncoderState = next;
  ++encoderTransitionEvents;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      ++encoderTransitionCount;
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      --encoderTransitionCount;
      break;
    default:
      encoderTransitionCount = 0;
      break;
  }

  if (encoderTransitionCount >= 4) {
    encoderTransitionCount = 0;
    if (encoderPendingSteps < 24) {
      ++encoderPendingSteps;
    }
  } else if (encoderTransitionCount <= -4) {
    encoderTransitionCount = 0;
    if (encoderPendingSteps > -24) {
      --encoderPendingSteps;
    }
  }
}

void handleEncoderStep(int direction) {
  lastActivityMs = millis();
  pendingShortPress = false;
  if (sleeping) {
    wakeFromSleep("ENC");
    return;
  }
  if (calState != CAL_IDLE) {
    return;
  }

  if (menuActive) {
    moveMenu(direction);
    return;
  }

  if (tuneMode) {
    int voltageDirection = currentBendDirection() == kPicoBendDown ? -direction : direction;
    nudgeToeMapMillivolts(voltageDirection * static_cast<int>(tuneStepMv));
    return;
  }

  if (outputFocusIsLfoOut()) {
    if (currentLfoOutIndex() == 1 && lfo2Linked()) {
      // Linked LFO2 ignores its own rate; the encoder walks the phase offset.
      setLfo2PhaseOffset(
          static_cast<uint8_t>((settings.lfo2PhaseOffset + kPicoLfoPhaseOffsetCount +
                                (direction > 0 ? 1 : kPicoLfoPhaseOffsetCount - 1)) %
                               kPicoLfoPhaseOffsetCount),
          true);
      return;
    }
    setLfoOutRateStep(currentLfoOutIndex(),
                      static_cast<int>(lfoOutRateStep(currentLfoOutIndex())) + direction,
                      true);
    return;
  }

  if (outputModeIsLfo()) {
    setLfoDepthPercent(static_cast<int>(settings.lfoDepthPercent) +
                           direction * static_cast<int>(kPicoLfoDepthStepPercent),
                       true);
    return;
  }

  setIntervalMagnitude(static_cast<int>(currentIntervalMagnitude()) + direction, true);
}

void pollEncoder() {
  noInterrupts();
  int8_t pendingSteps = encoderPendingSteps;
  encoderPendingSteps = 0;
  uint32_t transitionEvents = encoderTransitionEvents;
  encoderTransitionEvents = 0;
  uint8_t state = lastEncoderState;
  int8_t transitionCount = encoderTransitionCount;
  interrupts();

  if (encoderDebug && transitionEvents > 0) {
    Serial.print(F("ENC transitions="));
    Serial.print(transitionEvents);
    Serial.print(F(" state="));
    Serial.print(state, BIN);
    Serial.print(F(" count="));
    Serial.print(transitionCount);
    Serial.print(F(" steps="));
    Serial.print(pendingSteps);
    Serial.print(F(" A14="));
    Serial.print(digitalRead(ENCODER_A_PIN) == LOW ? F("LOW") : F("HIGH"));
    Serial.print(F(" B15="));
    Serial.println(digitalRead(ENCODER_B_PIN) == LOW ? F("LOW") : F("HIGH"));
  }

  while (pendingSteps > 0) {
    handleEncoderStep(1);
    --pendingSteps;
  }
  while (pendingSteps < 0) {
    handleEncoderStep(-1);
    ++pendingSteps;
  }
}

void pollEncoderButton() {
  bool rawPressed = digitalRead(ENCODER_SWITCH_PIN) == LOW;
  uint32_t now = millis();

  if (rawPressed != lastButtonRawPressed) {
    lastButtonRawPressed = rawPressed;
    lastButtonChangeMs = now;
  }

  if (rawPressed != stableButtonPressed && now - lastButtonChangeMs >= BUTTON_DEBOUNCE_MS) {
    stableButtonPressed = rawPressed;

    if (buttonDebug) {
      Serial.print(F("BTN "));
      Serial.println(stableButtonPressed ? F("press") : F("release"));
    }

    if (stableButtonPressed) {
      lastActivityMs = now;
      buttonPressStartedMs = now;
      buttonLongHandled = false;
      buttonIgnoreRelease = false;

      if (sleeping) {
        wakeFromSleep("BUTTON");
        buttonLongHandled = true;
        buttonIgnoreRelease = true;
      }
    } else {
      buttonReleasedSinceBoot = true;
      uint32_t heldMs = now - buttonPressStartedMs;

      if (buttonDebug) {
        Serial.print(F("BTN heldMs="));
        Serial.println(heldMs);
      }

      if (!buttonIgnoreRelease && !buttonLongHandled) {
        handleShortPress();
      }

      buttonLongHandled = false;
      buttonIgnoreRelease = false;
    }
  }

  if (stableButtonPressed && !buttonLongHandled && !buttonIgnoreRelease &&
      !sleeping && calState == CAL_IDLE && !tuneMode &&
      now - buttonPressStartedMs >= MENU_LONG_PRESS_MS) {
    buttonLongHandled = true;
    buttonIgnoreRelease = true;
    if (buttonDebug) {
      Serial.println(menuActive ? F("BTN long -> menu exit") : F("BTN long -> menu open"));
    }
    if (menuActive) {
      closeMenu();
    } else {
      openMenu();
    }
  }
}

void pollSerialCommand(char* line) {
  char* command = strtok(line, " \t\r\n");
  if (command == nullptr) {
    return;
  }

  for (char* p = command; *p; ++p) {
    if (*p >= 'A' && *p <= 'Z') {
      *p = static_cast<char>(*p - 'A' + 'a');
    }
  }

  if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printHelp();
  } else if (strcmp(command, "status") == 0) {
    printStatus();
  } else if (strcmp(command, "monitor") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "on") == 0) {
      settings.monitorEnabled = 1;
      markSettingsDirty();
      Serial.println(F("OK monitor on"));
    } else if (arg != nullptr && strcmp(arg, "off") == 0) {
      settings.monitorEnabled = 0;
      markSettingsDirty();
      Serial.println(F("OK monitor off"));
    } else {
      Serial.println(F("ERR monitor on|off"));
    }
	  } else if (strcmp(command, "interval") == 0 || strcmp(command, "semi") == 0) {
	    char* arg = strtok(nullptr, " \t\r\n");
	    if (arg == nullptr) {
	      Serial.println(F("ERR interval -9..9"));
	    } else {
	      setSemitones(atoi(arg), true);
	    }
	  } else if (strcmp(command, "up") == 0) {
	    setBendDirection(kPicoBendUp, true);
	  } else if (strcmp(command, "down") == 0) {
	    setBendDirection(kPicoBendDown, true);
	  } else if (strcmp(command, "dir") == 0 || strcmp(command, "direction") == 0) {
	    char* arg = strtok(nullptr, " \t\r\n");
	    if (arg != nullptr && strcmp(arg, "up") == 0) {
	      setBendDirection(kPicoBendUp, true);
	    } else if (arg != nullptr && (strcmp(arg, "down") == 0 || strcmp(arg, "dn") == 0)) {
	      setBendDirection(kPicoBendDown, true);
	    } else if (arg != nullptr && strcmp(arg, "toggle") == 0) {
	      toggleBendDirection();
	    } else {
	      Serial.println(F("ERR direction up|down|toggle"));
	    }
  } else if (strcmp(command, "pol") == 0 || strcmp(command, "polarity") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (!outputFocusIsLfoOut()) {
      Serial.println(F("ERR polarity applies after focus lfo1|lfo2; use direction for EXP"));
    } else if (arg != nullptr && strcmp(arg, "up") == 0) {
      setLfoOutPolarity(currentLfoOutIndex(), static_cast<uint8_t>(kPicoBendUp), true);
    } else if (arg != nullptr && (strcmp(arg, "down") == 0 || strcmp(arg, "dn") == 0)) {
      setLfoOutPolarity(currentLfoOutIndex(), static_cast<uint8_t>(kPicoBendDown), true);
    } else if (arg != nullptr && strcmp(arg, "toggle") == 0) {
      toggleFocusedPolarityOrDirection();
    } else {
      Serial.println(F("ERR polarity up|down|toggle"));
    }
	  } else if (strcmp(command, "more") == 0 || strcmp(command, "inc") == 0) {
	    setIntervalMagnitude(static_cast<int>(currentIntervalMagnitude()) + 1, true);
	  } else if (strcmp(command, "less") == 0 || strcmp(command, "dec") == 0) {
	    setIntervalMagnitude(static_cast<int>(currentIntervalMagnitude()) - 1, true);
	  } else if (strcmp(command, "center") == 0) {
	    char* arg = strtok(nullptr, " \t\r\n");
	    if (arg == nullptr) {
	      setIntervalMagnitude(0, true);
	    } else {
	      Serial.println(F("ERR center voltage removed; use direction up|down and range 0..3300"));
	    }
	  } else if (strcmp(command, "octave") == 0 || strcmp(command, "range") == 0) {
	    char* arg = strtok(nullptr, " \t\r\n");
	    if (arg == nullptr) {
	      Serial.println(F("ERR range 0..3300"));
	    } else {
	      setOctaveMillivolts(atoi(arg));
	    }
  } else if (strcmp(command, "focus") == 0 || strcmp(command, "out") == 0 ||
             strcmp(command, "target") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    uint8_t focus = 0;
    if (parseOutputFocus(arg, &focus)) {
      setOutputFocus(focus, true);
    } else if (arg != nullptr && strcmp(arg, "next") == 0) {
      cycleOutputFocus();
    } else {
      Serial.println(F("ERR focus exp|lfo1|lfo2|next"));
    }
  } else if (strcmp(command, "mode") == 0 || strcmp(command, "output") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    uint8_t mode = 0;
    if (parseOutputMode(arg, &mode)) {
      if (outputFocusIsLfoOut() && mode == kPicoOutputPedal) {
        Serial.println(F("ERR LFO outputs support mode lo|fm"));
      } else {
        setFocusedOutputMode(mode, true);
      }
    } else {
      Serial.println(outputFocusIsLfoOut() ? F("ERR mode lo|fm") : F("ERR mode ped|lo|fm"));
    }
  } else if (strcmp(command, "wave") == 0 || strcmp(command, "lfo") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    uint8_t wave = 0;
    if (parseLfoWave(arg, &wave)) {
      setFocusedLfoWave(wave, true);
    } else {
      Serial.println(F("ERR wave sine|tri|sawup|sawdown|square|pulse|sh|drift"));
    }
  } else if (strcmp(command, "rate") == 0 || strcmp(command, "speed") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg == nullptr) {
      Serial.println(F("ERR rate <hz>|step <n>"));
    } else if (outputFocusIsExp() && !outputModeIsLfo()) {
      Serial.println(F("ERR rate only applies to EXP in mode lo|fm; use focus lfo1 or focus lfo2 for dedicated LFOs"));
    } else if (strcmp(arg, "step") == 0) {
      char* value = strtok(nullptr, " \t\r\n");
      if (value == nullptr) {
        Serial.println(F("ERR rate step <n>"));
      } else {
        setFocusedLfoRateStep(atoi(value), true);
      }
    } else {
      setFocusedLfoRateHz(atof(arg), true);
    }
  } else if (strcmp(command, "sync") == 0 || strcmp(command, "phase") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "all") == 0) {
      resetAllLfoPhases();
    } else {
      resetFocusedLfoPhase();
    }
  } else if (strcmp(command, "tap") == 0) {
    if (outputFocusIsExp() && !outputModeIsLfo()) {
      Serial.println(F("ERR tap applies to a focused LFO; use focus lfo1|lfo2 or mode lo|fm"));
    } else if (applyTapTempo()) {
      resetFocusedLfoPhase();
    } else {
      resetFocusedLfoPhase();
      Serial.println(F("Tap armed; send tap again at the target tempo"));
    }
  } else if (strcmp(command, "depth") == 0 || strcmp(command, "atten") == 0 ||
             strcmp(command, "level") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg == nullptr) {
      Serial.println(F("ERR depth 0..100"));
    } else {
      setFocusedLfoDepthPercent(atoi(arg), true);
    }
  } else if (strcmp(command, "pw") == 0 || strcmp(command, "pulsewidth") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg == nullptr) {
      Serial.println(F("ERR pw 5..95"));
    } else {
      setFocusedLfoPulseWidth(atoi(arg), true);
    }
  } else if (strcmp(command, "offset") == 0 || strcmp(command, "ofs") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (!outputFocusIsLfoOut()) {
      Serial.println(F("ERR offset applies after focus lfo1|lfo2"));
    } else if (arg == nullptr) {
      Serial.println(F("ERR offset -50..50"));
    } else {
      setLfoOutOffsetPercent(currentLfoOutIndex(), atoi(arg), true);
    }
  } else if (strcmp(command, "link") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg == nullptr) {
      Serial.println(F("ERR link off|1:1|1:2|1:4|3:2|2:1|4:1|phase <0|90|180|270>"));
    } else if (strcmp(arg, "phase") == 0) {
      char* value = strtok(nullptr, " \t\r\n");
      int degrees = value != nullptr ? atoi(value) : -1;
      if (degrees == 0 || degrees == 90 || degrees == 180 || degrees == 270) {
        setLfo2PhaseOffset(static_cast<uint8_t>(degrees / 90), true);
      } else {
        Serial.println(F("ERR link phase 0|90|180|270"));
      }
    } else {
      uint8_t link = 0xff;
      if (strcmp(arg, "off") == 0) {
        link = kPicoLfoLinkOff;
      } else {
        for (uint8_t candidate = kPicoLfoLink1to1; candidate < kPicoLfoLinkCount; ++candidate) {
          if (strcmp(arg, picoLfoLinkName(candidate)) == 0) {
            link = candidate;
            break;
          }
        }
      }
      if (link != 0xff) {
        setLfo2Link(link, true);
      } else {
        Serial.println(F("ERR link off|1:1|1:2|1:4|3:2|2:1|4:1|phase <deg>"));
      }
    }
  } else if (strcmp(command, "clock") == 0 || strcmp(command, "clk") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "off") == 0) {
      setClockSource(kPicoClockOff, true);
    } else if (arg != nullptr && strcmp(arg, "lfo1") == 0) {
      setClockSource(kPicoClockLfo1, true);
    } else if (arg != nullptr && strcmp(arg, "lfo2") == 0) {
      setClockSource(kPicoClockLfo2, true);
    } else {
      Serial.println(F("ERR clock off|lfo1|lfo2"));
    }
  } else if (strcmp(command, "response") == 0 || strcmp(command, "resp") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "off") == 0) {
      setResponseCents(0);
    } else if (arg != nullptr) {
      setResponseCents(atoi(arg));
    } else {
      Serial.println(F("ERR response <fullScaleCents>|off"));
    }
  } else if (strcmp(command, "curve") == 0 || strcmp(command, "feel") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    uint8_t mode = 0;
    if (parseCurveMode(arg, &mode)) {
      setCurveMode(mode, true);
    } else {
      Serial.println(F("ERR curve linear|easeout|square|smooth"));
    }
  } else if (strcmp(command, "map") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "show") == 0) {
      printToeMap();
    } else if (arg != nullptr && strcmp(arg, "reset") == 0) {
      resetToeMapToLinear();
    } else if (arg != nullptr) {
      char* value = strtok(nullptr, " \t\r\n");
      if (value == nullptr) {
        Serial.println(F("ERR map <semitones> <toeMv>|show|reset"));
      } else {
        setToeMapMillivolts(atoi(arg), atoi(value), true);
      }
    } else {
      Serial.println(F("ERR map <semitones> <toeMv>|show|reset"));
    }
  } else if (strcmp(command, "toe") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg == nullptr) {
      Serial.println(F("ERR toe 0..3300"));
    } else {
      setToeMapMillivolts(settings.semitones, atoi(arg), true);
    }
  } else if (strcmp(command, "nudge") == 0 || strcmp(command, "trim") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg == nullptr) {
      Serial.println(F("ERR nudge +/-mv"));
    } else {
      nudgeToeMapMillivolts(atoi(arg));
    }
  } else if (strcmp(command, "tune") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    char* value = strtok(nullptr, " \t\r\n");
    if (arg == nullptr || strcmp(arg, "on") == 0) {
      setTuneMode(true);
    } else if (strcmp(arg, "off") == 0) {
      setTuneMode(false);
    } else if (strcmp(arg, "step") == 0 && value != nullptr) {
      setTuneStepMillivolts(atoi(value));
    } else {
      Serial.println(F("ERR tune [on|off|step <mv>]"));
    }
  } else if (strcmp(command, "cv") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "off") == 0) {
      clearCvOverride();
    } else if (arg != nullptr) {
      setCvOverrideMillivolts(atoi(arg));
    } else {
      Serial.println(F("ERR cv <mv>|off"));
    }
  } else if (strcmp(command, "save") == 0) {
    saveSettings();
  } else if (strcmp(command, "defaults") == 0) {
    settings = makeDefaultPicoSettings();
    configureProcessor();
    markSettingsDirty();
    saveSettings();
    Serial.println(F("OK defaults restored"));
  } else if (strcmp(command, "invert") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "on") == 0) {
      settings.invert = 1;
    } else if (arg != nullptr && strcmp(arg, "off") == 0) {
      settings.invert = 0;
    } else {
      Serial.println(F("ERR invert on|off"));
      return;
    }
    configureProcessor();
    markSettingsDirty();
    Serial.println(settings.invert ? F("OK invert on") : F("OK invert off"));
  } else if (strcmp(command, "cal") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "start") == 0) {
      startCalibration();
    } else if (arg != nullptr && strcmp(arg, "heel") == 0) {
      captureHeel();
    } else if (arg != nullptr && strcmp(arg, "toe") == 0) {
      captureToe();
    } else if (arg != nullptr && strcmp(arg, "reset") == 0) {
      resetCalibration();
    } else {
      Serial.println(F("ERR cal start|heel|toe|reset"));
    }
  } else if (strcmp(command, "start") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "cal") == 0) {
      startCalibration();
    } else {
      Serial.println(F("ERR start cal"));
    }
  } else if (strcmp(command, "menu") == 0) {
    openMenu();
  } else if (strcmp(command, "sleep") == 0) {
    enterSleep("SERIAL");
  } else if (strcmp(command, "wake") == 0) {
    wakeFromSleep("SERIAL");
  } else if (strcmp(command, "dac") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "probe") == 0) {
      dacReady = probeI2c(MCP4728_ADDRESS);
      Serial.println(dacReady ? F("MCP4728 PASS at 0x60") : F("MCP4728 FAIL at 0x60"));
      for (uint8_t channel = 0; channel < DAC_CHANNEL_COUNT; ++channel) {
        lastWrittenDacCodes[channel] = -1;
      }
      refreshControlNow();
    } else if (arg != nullptr && strcmp(arg, "eeprom") == 0) {
      if (!dacReady) {
        Serial.println(F("ERR dac not ready; run dac probe first"));
      } else if (programMcp4728PowerOnDefaults()) {
        Serial.println(F("OK MCP4728 power-on EEPROM set to 0V on all channels"));
        for (uint8_t channel = 0; channel < DAC_CHANNEL_COUNT; ++channel) {
          lastWrittenDacCodes[channel] = -1;
        }
        refreshControlNow();
      } else {
        Serial.println(F("ERR MCP4728 EEPROM write failed"));
      }
    } else {
      Serial.println(F("ERR dac probe|eeprom"));
    }
  } else if (strcmp(command, "display") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "probe") == 0) {
      displayReady = initDisplay();
      Serial.println(displayReady ? F("SSD1306 PASS at 0x3C") : F("SSD1306 FAIL at 0x3C"));
    } else {
      Serial.println(F("ERR display probe"));
    }
  } else if (strcmp(command, "enc") == 0 || strcmp(command, "encoder") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    char* value = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "debug") == 0 && value != nullptr &&
        strcmp(value, "on") == 0) {
      encoderDebug = true;
      Serial.println(F("OK encoder debug on; turn knob slowly"));
      printStatus();
    } else if (arg != nullptr && strcmp(arg, "debug") == 0 && value != nullptr &&
               strcmp(value, "off") == 0) {
      encoderDebug = false;
      Serial.println(F("OK encoder debug off"));
    } else {
      Serial.println(F("ERR enc debug on|off"));
    }
  } else if (strcmp(command, "btn") == 0 || strcmp(command, "button") == 0) {
    char* arg = strtok(nullptr, " \t\r\n");
    char* value = strtok(nullptr, " \t\r\n");
    if (arg != nullptr && strcmp(arg, "debug") == 0 && value != nullptr &&
        strcmp(value, "on") == 0) {
      buttonDebug = true;
      Serial.println(F("OK button debug on; press and hold encoder"));
      printStatus();
    } else if (arg != nullptr && strcmp(arg, "debug") == 0 && value != nullptr &&
               strcmp(value, "off") == 0) {
      buttonDebug = false;
      Serial.println(F("OK button debug off"));
    } else {
      Serial.println(F("ERR btn debug on|off"));
    }
  } else {
    Serial.println(F("ERR unknown command; type help"));
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        pollSerialCommand(serialLine);
        serialLineLength = 0;
      }
      continue;
    }

    if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = c;
    } else {
      serialLineLength = 0;
      Serial.println(F("ERR line too long"));
    }
  }
}

void printHelp() {
  Serial.println(F("Therevox Expression Controller commands:"));
  Serial.println(F("  status             show current state"));
  Serial.println(F("  monitor on|off      periodic serial monitor"));
  Serial.println(F("  interval -9..9      signed interval; max label is 6"));
  Serial.println(F("  direction up|down   one-way bend direction"));
  Serial.println(F("  up|down             aliases for direction up|down"));
  Serial.println(F("  more|less           change interval size by one semitone"));
  Serial.println(F("  center              set interval to unison"));
  Serial.println(F("  range <mv>          old linear voltage scale, 0..3300; disables response fit"));
  Serial.println(F("  focus exp|lfo1|lfo2 choose which output encoder/menu/serial edits"));
  Serial.println(F("  focus next          cycle EXP -> LFO1 -> LFO2"));
  Serial.println(F("  mode ped|lo|fm      focused EXP: PED interval, LO slow LFO, FM fast LFO"));
  Serial.println(F("  mode lo|fm          focused LFO1/LFO2: slow or fast dedicated LFO"));
  Serial.println(F("  wave sine|tri|sawup|sawdown|square|pulse|sh|drift for focused LFO"));
  Serial.println(F("  rate <hz>           focused EXP LFO toe/max speed or focused LFO fixed speed"));
  Serial.println(F("  rate step <n>       set raw focused LFO speed step"));
  Serial.println(F("  depth 0..100        set focused LFO depth/attenuation in 5% steps"));
  Serial.println(F("  pw 5..95            set focused LFO pulse-wave width"));
  Serial.println(F("  offset -50..50      shift focused LFO1/LFO2 center voltage"));
  Serial.println(F("  link off|1:1|1:2|1:4|3:2|2:1|4:1  lock LFO2 rate to LFO1"));
  Serial.println(F("  link phase 0|90|180|270  linked LFO2 phase offset"));
  Serial.println(F("  clock off|lfo1|lfo2 full-swing clock square on DAC channel D"));
  Serial.println(F("  sync                reset focused LFO phase"));
  Serial.println(F("  sync all            reset every LFO phase together"));
  Serial.println(F("  tap                 send twice at tempo to set focused LFO rate"));
  Serial.println(F("  polarity up|down    focused LFO1/LFO2 polarity"));
  Serial.println(F("  response <cents>|off full-scale cents; 3960 = standard 1V/oct, bench default 924"));
  Serial.println(F("  curve linear|easeout|square|smooth"));
  Serial.println(F("  map show|reset      show or rebuild directional toe-voltage map"));
  Serial.println(F("  map <semi> <mv>     set toe voltage for one interval"));
  Serial.println(F("  toe <mv>            set toe voltage for current interval"));
  Serial.println(F("  nudge <+/-mv>       adjust current interval toe voltage"));
  Serial.println(F("  tune [on|off]       encoder adjusts current interval toe mV; short press saves/exits"));
  Serial.println(F("  tune step <mv>      set tune-mode encoder step size, default 10mV"));
  Serial.println(F("  cv <mv>|off         fixed DAC output for hard Therevox range tests"));
  Serial.println(F("  cal start           start encoder calibration flow"));
  Serial.println(F("  cal heel|toe        capture endpoint from serial"));
  Serial.println(F("  cal reset           reset endpoints to raw 0..4095"));
  Serial.println(F("  invert on|off       reverse pedal travel"));
  Serial.println(F("  dac probe           retry MCP4728 detection"));
  Serial.println(F("  dac eeprom          program 0V power-on defaults into the MCP4728"));
  Serial.println(F("  display probe       retry OLED detection"));
  Serial.println(F("  enc debug on|off    print rotary encoder A/B transitions"));
  Serial.println(F("  btn debug on|off    print encoder button press timing"));
  Serial.println(F("  menu                open OLED menu"));
  Serial.println(F("  sleep|wake          manual sleep test; auto idle sleep disabled in bench build"));
  Serial.println(F("  save                persist settings"));
  Serial.println(F("Encoder: double=focus EXP/LFO1/LFO2. EXP turn=interval/depth; LFO1/2 turn=speed. Short=unison/sync/capture."));
  Serial.println(F("Menu: OUT selects focus. WAVE/DEPTH show for LFO modes; CURVE/CAL show for EXP; DIR/POL toggles direction/polarity."));
  Serial.println(F("Tune mode: turn = toe mV trim, short press = save/exit."));
}

void updateControl() {
  currentRaw = readPedalRaw();
  pedalState = pedalProcessor.process(currentRaw, 1000.0f);

  if (abs(static_cast<int>(currentRaw) - static_cast<int>(lastActivityRaw)) > PEDAL_ACTIVITY_RAW_DELTA) {
    lastActivityRaw = currentRaw;
    lastActivityMs = millis();
    if (sleeping) {
      wakeFromSleep("PEDAL");
    }
  }

  updateOutputFromPedalState();
  updateLfoOutStates();

  if (!sleeping) {
    writeOutputToDac();
  }
}

void printMonitor() {
  printStatus();
}

} // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 1500) {
    delay(10);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SWITCH_PIN, INPUT_PULLUP);
  buttonReleasedSinceBoot = digitalRead(ENCODER_SWITCH_PIN) == HIGH;

  analogReadResolution(ADC_BITS);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  loadSettings();
  configureProcessor();

  currentRaw = readPedalRaw();
  lastActivityRaw = currentRaw;
  lastActivityMs = millis();
  pedalState = pedalProcessor.process(currentRaw, 1000.0f);
  updateOutputFromPedalState();
  updateLfoOutStates();

  lastEncoderState = readEncoderState();
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), handleEncoderInterrupt, CHANGE);

  dacReady = probeI2c(MCP4728_ADDRESS);
  displayReady = initDisplay();
  if (dacReady) {
    writeOutputToDac(true);
  }

  Serial.println();
  Serial.println(F("Therevox Expression Controller"));
  Serial.println(F("Board: Raspberry Pi Pico H"));
  Serial.println(dacReady ? F("MCP4728 PASS at 0x60") : F("MCP4728 FAIL at 0x60"));
  Serial.println(displayReady ? F("SSD1306 PASS at 0x3C") : F("SSD1306 FAIL at 0x3C"));
  if (!buttonReleasedSinceBoot) {
    Serial.println(F("WARN: encoder switch GP13 reads PRESSED at boot; check switch wiring"));
  }
  Serial.println(EXPCTRL_PICO_USE_EEPROM ? F("Settings: EEPROM flash persistence enabled") : F("Settings: RAM only"));
  printHelp();
  printStatus();

  setDisplayMessage("PICO", dacReady ? "DAC READY" : "DAC FAIL", CAL_MESSAGE_MS);
  renderDisplay();
}

void loop() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  pollSerial();
  pollEncoder();
  pollEncoderButton();
  pollPendingShortPress();

  if (static_cast<uint32_t>(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    lastControlUs += CONTROL_PERIOD_US;
    updateControl();
  }

  if (!sleeping && settings.monitorEnabled && nowMs - lastMonitorMs >= MONITOR_MS) {
    lastMonitorMs = nowMs;
    printMonitor();
  }

  if (!sleeping && nowMs - lastDisplayMs >= DISPLAY_MS) {
    lastDisplayMs = nowMs;
    renderDisplay();
  }

  if (IDLE_SLEEP_ENABLED &&
      !sleeping && calState == CAL_IDLE && !tuneMode && !cvOverrideEnabled &&
      nowMs - lastActivityMs >= IDLE_SLEEP_MS) {
    enterSleep("IDLE");
  }

  if (!sleeping && calState == CAL_IDLE && settingsDirty &&
      nowMs - lastSettingsChangeMs >= SETTINGS_AUTOSAVE_MS) {
    saveSettings();
  }

  if (nowMs - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = nowMs;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
