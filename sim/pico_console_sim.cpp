#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../pico/PrecisionExpressionControllerPico/PicoFirmwareCore.h"

using namespace expctrl;

namespace {

constexpr uint16_t kAdcMax = 4095;
constexpr bool kIdleSleepEnabled = false;
constexpr uint32_t kIdleSleepMs = 60UL * 60UL * 1000UL;
constexpr uint32_t kAutosaveMs = 3000;
constexpr uint16_t kPedalActivityRawDelta = 8;
constexpr uint16_t kMaxOutputMv = static_cast<uint16_t>(kPicoDacFullScaleMicrovolts / 1000L);
constexpr uint16_t kDefaultTuneStepMv = 10;
constexpr uint8_t kLfoOutCount = 2;

enum OutputFocus {
  OutputFocusExp = 0,
  OutputFocusLfo1 = 1,
  OutputFocusLfo2 = 2,
  OutputFocusCount = 3,
};

enum CalibrationState {
  CalIdle,
  CalWaitHeel,
  CalWaitToe,
};

enum MenuItem {
  MenuOut,
  MenuMode,
  MenuWave,
  MenuDepth,
  MenuPw,
  MenuOfs,
  MenuLink,
  MenuPhs,
  MenuClk,
  MenuCurve,
  MenuCal,
  MenuDir,
  MenuDone,
  MenuCount,
};

enum MenuEditField {
  MenuEditNone,
  MenuEditOut,
  MenuEditMode,
  MenuEditWave,
  MenuEditDepth,
  MenuEditPw,
  MenuEditOfs,
  MenuEditLink,
  MenuEditPhs,
  MenuEditClk,
  MenuEditCurve,
};

constexpr uint32_t kLfoSeedExp = 0xE0u;
constexpr uint32_t kLfoSeedOut[kLfoOutCount] = {0xA1u, 0xB2u};
constexpr uint32_t kTapMinMs = 450;
constexpr uint32_t kTapMaxMs = 5000;

struct Settings {
  uint16_t heelRaw = 0;
  uint16_t toeRaw = kAdcMax;
  int8_t semitones = kPicoMaxSemitones;
  bool invert = false;
  PicoBendDirection bendDirection = kPicoBendUp;
  uint8_t curveMode = kPicoCurveLinear;
  uint8_t outputMode = kPicoOutputPedal;
  uint8_t lfoWave = kPicoLfoSine;
  uint8_t lfoLoRate = kPicoLfoLoRateSteps;
  uint8_t lfoFmRate = kPicoLfoFmRateSteps;
  uint8_t lfoDepthPercent = kPicoLfoMaxDepthPercent;
  uint8_t outputFocus = OutputFocusExp;
  uint8_t lfoOutMode[kLfoOutCount] = {kPicoOutputLfoLo, kPicoOutputLfoLo};
  uint8_t lfoOutWave[kLfoOutCount] = {kPicoLfoSine, kPicoLfoTriangle};
  uint8_t lfoOutLoRate[kLfoOutCount] = {kPicoDefaultLfoLoRate, kPicoDefaultLfoLoRate};
  uint8_t lfoOutFmRate[kLfoOutCount] = {kPicoDefaultLfoFmRate, kPicoDefaultLfoFmRate};
  uint8_t lfoOutDepthPercent[kLfoOutCount] = {kPicoLfoMaxDepthPercent,
                                              kPicoLfoMaxDepthPercent};
  PicoBendDirection lfoOutPolarity[kLfoOutCount] = {kPicoBendUp, kPicoBendUp};
  uint8_t lfoPulseWidth = kPicoDefaultLfoPulseWidthPercent;
  uint8_t lfoOutPulseWidth[kLfoOutCount] = {kPicoDefaultLfoPulseWidthPercent,
                                            kPicoDefaultLfoPulseWidthPercent};
  int8_t lfoOutOffsetPercent[kLfoOutCount] = {0, 0};
  uint8_t lfo2Link = kPicoLfoLinkOff;
  uint8_t lfo2PhaseOffset = kPicoLfoPhase0;
  uint8_t clockSource = kPicoClockOff;
  uint16_t rangeMv = 3300;
  uint16_t responseCents = kDefaultFullScaleResponseCents;
  uint16_t toeMapMv[kPicoSemitoneMapCount] = {};
};

struct Snapshot {
  int32_t outputMicrovolts = kDefaultCenterMicrovolts;
  int32_t lfoOutMicrovolts[kLfoOutCount] = {0, 0};
  uint16_t dacCode = 0;
  uint16_t lfoOutDacCode[kLfoOutCount] = {0, 0};
  uint16_t clockDacCode = 0;
  uint16_t raw = 0;
  float pedal = 0.0f;
};

class PicoConsoleSim {
 public:
  void boot(bool dacPresentValue = true, bool oledPresentValue = true) {
    dacPresent = dacPresentValue;
    oledPresent = oledPresentValue;
    fillToeMapFromCurrentFit();
    configureProcessor();
    currentRaw = 0;
    lastActivityRaw = 0;
    advanceMs(250);

    printf("BOOT Therevox Expression Controller SIM\n");
    printf("SERIAL: MCP4728 %s at 0x60\n", dacPresent ? "PASS" : "FAIL");
    printf("SERIAL: SSD1306 %s at 0x3C\n", oledPresent ? "PASS" : "FAIL");
    printStatus("boot");
  }

  void setPedalRaw(uint16_t raw, uint32_t settleMs = 700) {
    currentRaw = raw;
    advanceMs(settleMs);
  }

  void commandInterval(int semitones) {
    int clamped = clampPicoSemitones(semitones);
    if (clamped > 0) {
      settings.bendDirection = kPicoBendUp;
    } else if (clamped < 0) {
      settings.bendDirection = kPicoBendDown;
    }
    settings.semitones =
        signedPicoInterval(picoIntervalMagnitude(clamped), settings.bendDirection);
    markDirty();
    char label[8];
    signedIntervalLabel(label, sizeof(label));
    printf("SERIAL: OK interval %s\n", label);
  }

  void commandMap(int semitones, uint16_t toeMv) {
    int clamped = clampPicoSemitones(semitones);
    settings.toeMapMv[picoSemitoneMapIndex(clamped)] =
        static_cast<uint16_t>(clampValue<int>(toeMv, 0, kMaxOutputMv));
    markDirty();
    int8_t previousSemitones = settings.semitones;
    settings.semitones = clamped;
    char label[8];
    signedIntervalLabel(label, sizeof(label));
    settings.semitones = previousSemitones;
    printf("SERIAL: OK map %s toeMv=%u\n",
           label,
           settings.toeMapMv[picoSemitoneMapIndex(clamped)]);
  }

  void commandResponse(int cents) {
    if (cents <= 0) {
      settings.responseCents = 0;
      fillLinearToeMap();
      markDirty();
      printf("SERIAL: OK response off; linear voltage map restored\n");
      return;
    }

    settings.responseCents = static_cast<uint16_t>(clampValue<int>(cents, 100, 6000));
    fillResponseToeMap();
    markDirty();
    printf("SERIAL: OK responseCents %u; map rebuilt\n", settings.responseCents);
  }

  void commandTune(bool enabled) {
    tuneMode = enabled;
    if (tuneMode) {
      cvOverride = false;
    }
    printf("SERIAL: OK tune %s\n", tuneMode ? "on" : "off");
  }

  void commandTuneStep(uint16_t stepMv) {
    tuneStepMv = static_cast<uint16_t>(clampValue<int>(stepMv, 1, 250));
    printf("SERIAL: OK tune stepMv %u\n", tuneStepMv);
  }

  void commandCv(uint16_t mv) {
    cvOverride = true;
    cvOverrideMv = static_cast<uint16_t>(clampValue<int>(mv, 0, kMaxOutputMv));
    tuneMode = false;
    printf("SERIAL: OK cv fixed %umV\n", cvOverrideMv);
  }

  void commandCvOff() {
    cvOverride = false;
    printf("SERIAL: OK cv off\n");
  }

  void commandDirection(PicoBendDirection direction) {
    settings.bendDirection = direction;
    settings.semitones =
        signedPicoInterval(picoIntervalMagnitude(settings.semitones), settings.bendDirection);
    markDirty();
    printf("SERIAL: OK direction %s\n", directionLabel());
  }

  void commandFocus(uint8_t focus) {
    settings.outputFocus =
        static_cast<uint8_t>(clampValue<int>(focus, OutputFocusExp, OutputFocusCount - 1));
    markDirty();
    printf("SERIAL: OK focus %s\n", outputFocusLabel(settings.outputFocus));
  }

  void commandCurve(uint8_t curveMode) {
    settings.curveMode = clampPicoCurveMode(curveMode);
    configureProcessor();
    markDirty();
    printf("SERIAL: OK curve %s\n", picoCurveName(settings.curveMode));
  }

  void commandMode(uint8_t outputMode) {
    if (outputFocusIsLfoOut()) {
      uint8_t index = currentLfoOutIndex();
      settings.lfoOutMode[index] = clampPicoLfoOutputMode(outputMode);
      lfoOutPhase[index] = 0.0f;
    } else {
      settings.outputMode = clampPicoOutputMode(outputMode);
      lfoPhase = 0.0f;
    }
    tuneMode = false;
    markDirty();
    printf("SERIAL: OK mode %s\n", picoOutputModeName(currentFocusedOutputMode()));
  }

  void commandWave(uint8_t wave) {
    if (outputFocusIsLfoOut()) {
      uint8_t index = currentLfoOutIndex();
      settings.lfoOutWave[index] = clampPicoLfoWave(wave);
      lfoOutPhase[index] = 0.0f;
    } else {
      settings.lfoWave = clampPicoLfoWave(wave);
      lfoPhase = 0.0f;
    }
    markDirty();
    printf("SERIAL: OK wave %s\n", picoLfoWaveName(currentFocusedLfoWave()));
  }

  void commandRateStep(int step) {
    if (outputFocusIsLfoOut()) {
      uint8_t index = currentLfoOutIndex();
      if (settings.lfoOutMode[index] == kPicoOutputLfoFm) {
        settings.lfoOutFmRate[index] = clampPicoLfoRateStep(step, settings.lfoOutMode[index]);
      } else {
        settings.lfoOutLoRate[index] = clampPicoLfoRateStep(step, settings.lfoOutMode[index]);
      }
    } else {
      if (settings.outputMode == kPicoOutputLfoFm) {
        settings.lfoFmRate = clampPicoLfoRateStep(step, settings.outputMode);
      } else {
        settings.lfoLoRate = clampPicoLfoRateStep(step, settings.outputMode);
      }
    }
    markDirty();
    printf("SERIAL: OK rate %.3fHz step=%u\n", currentFocusedLfoRateHz(), currentFocusedLfoRateStep());
  }

  void commandDepth(int percent) {
    if (outputFocusIsLfoOut()) {
      settings.lfoOutDepthPercent[currentLfoOutIndex()] = clampPicoLfoDepthPercent(percent);
    } else {
      settings.lfoDepthPercent = clampPicoLfoDepthPercent(percent);
    }
    markDirty();
    printf("SERIAL: OK depth %u%%\n", currentFocusedLfoDepthPercent());
  }

  void commandPulseWidth(int percent) {
    if (outputFocusIsLfoOut()) {
      settings.lfoOutPulseWidth[currentLfoOutIndex()] = clampPicoLfoPulseWidth(percent);
    } else {
      settings.lfoPulseWidth = clampPicoLfoPulseWidth(percent);
    }
    markDirty();
    printf("SERIAL: OK pw %u%%\n",
           outputFocusIsLfoOut() ? settings.lfoOutPulseWidth[currentLfoOutIndex()]
                                 : settings.lfoPulseWidth);
  }

  void commandOffset(int percent) {
    if (!outputFocusIsLfoOut()) {
      printf("SERIAL: ERR offset applies after focus lfo1|lfo2\n");
      return;
    }
    settings.lfoOutOffsetPercent[currentLfoOutIndex()] = clampPicoLfoOffsetPercent(percent);
    markDirty();
    printf("SERIAL: OK offset %+d%%\n",
           static_cast<int>(settings.lfoOutOffsetPercent[currentLfoOutIndex()]));
  }

  void commandLink(uint8_t link) {
    settings.lfo2Link = clampPicoLfoLink(link);
    markDirty();
    printf("SERIAL: OK LFO2 link %s\n", picoLfoLinkName(settings.lfo2Link));
  }

  void commandLinkPhase(uint8_t phaseOffset) {
    settings.lfo2PhaseOffset = clampPicoLfoPhaseOffset(phaseOffset);
    markDirty();
    printf("SERIAL: OK LFO2 link phase %u\n",
           picoLfoPhaseOffsetDegrees(settings.lfo2PhaseOffset));
  }

  void commandClock(uint8_t source) {
    settings.clockSource = clampPicoClockSource(source);
    markDirty();
    printf("SERIAL: OK clock %s\n", picoClockSourceName(settings.clockSource));
  }

  void commandSyncAll() {
    lfoPhase = 0.0f;
    lfoCycle = 0;
    for (uint8_t i = 0; i < kLfoOutCount; ++i) {
      lfoOutPhase[i] = 0.0f;
      lfoOutCycle[i] = 0;
    }
    printf("SERIAL: OK all LFO phases reset\n");
  }

  void commandTap() {
    if (outputFocusIsExp() && !outputModeIsLfo()) {
      printf("SERIAL: ERR tap applies to a focused LFO\n");
      return;
    }
    if (applyTapTempo()) {
      printf("SERIAL: OK tap rate %.3fHz\n", currentFocusedLfoRateHz());
    } else {
      printf("SERIAL: Tap armed\n");
    }
    syncFocusedLfo();
  }

  void turnEncoder(int detents) {
    int direction = detents >= 0 ? 1 : -1;
    for (int i = 0; i < abs(detents); ++i) {
      if (sleeping) {
        wake("ENC");
        continue;
      }
      if (calState != CalIdle) {
        continue;
      }
      if (menuActive) {
        if (menuEditField != MenuEditNone) {
          menuEditValue = wrapMenuChoice(
              static_cast<int>(menuEditValue) + direction,
              menuEditValueCount(menuEditField));
        } else {
          for (int attempt = 0; attempt < MenuCount; ++attempt) {
            menuIndex = static_cast<MenuItem>(
                wrapMenuChoice(static_cast<int>(menuIndex) + direction, MenuCount));
            if (menuItemVisible(menuIndex)) {
              break;
            }
          }
        }
        continue;
      }
      if (tuneMode) {
        int semitones = clampPicoSemitones(settings.semitones);
        uint16_t currentMv = settings.toeMapMv[picoSemitoneMapIndex(semitones)];
        int voltageDirection = settings.bendDirection == kPicoBendDown ? -direction : direction;
        settings.toeMapMv[picoSemitoneMapIndex(semitones)] =
            static_cast<uint16_t>(clampValue<int>(
                static_cast<int>(currentMv) + voltageDirection * static_cast<int>(tuneStepMv),
                0,
                kMaxOutputMv));
        markDirty();
        continue;
      }
      if (outputFocusIsLfoOut()) {
        uint8_t index = currentLfoOutIndex();
        if (index == 1 && settings.lfo2Link != kPicoLfoLinkOff) {
          commandLinkPhase(static_cast<uint8_t>(
              (settings.lfo2PhaseOffset + kPicoLfoPhaseOffsetCount +
               (direction > 0 ? 1 : kPicoLfoPhaseOffsetCount - 1)) %
              kPicoLfoPhaseOffsetCount));
          continue;
        }
        commandRateStep(static_cast<int>(lfoOutRateStep(index)) + direction);
        continue;
      }
      if (outputModeIsLfo()) {
        commandDepth(static_cast<int>(settings.lfoDepthPercent) +
                     direction * static_cast<int>(kPicoLfoDepthStepPercent));
        continue;
      }
      int nextMagnitude = static_cast<int>(picoIntervalMagnitude(settings.semitones)) + direction;
      settings.semitones = signedPicoInterval(
          static_cast<uint8_t>(clampValue<int>(nextMagnitude, 0, kPicoMaxSemitones)),
          settings.bendDirection);
      markDirty();
    }

    char label[8];
    signedIntervalLabel(label, sizeof(label));
    if (tuneMode) {
      printf("ENCODER: turn %+d detents -> %s toeMv=%u\n",
             detents,
             label,
             settings.toeMapMv[picoSemitoneMapIndex(settings.semitones)]);
    } else if (menuActive) {
      if (menuEditField != MenuEditNone) {
        printf("ENCODER: turn %+d detents -> %s %s\n",
               detents,
               menuEditTitle(menuEditField),
               menuEditValueLabel(menuEditField, menuEditValue));
      } else {
        printf("ENCODER: turn %+d detents -> menu %s\n", detents, menuItemTitle(menuIndex));
      }
    } else if (outputFocusIsLfoOut()) {
      printf("ENCODER: turn %+d detents -> %s rate %.3fHz\n",
             detents,
             outputFocusLabel(settings.outputFocus),
             lfoOutRateHz(currentLfoOutIndex()));
    } else if (outputModeIsLfo()) {
      printf("ENCODER: turn %+d detents -> depth %u%%\n",
             detents,
             settings.lfoDepthPercent);
    } else {
      printf("ENCODER: turn %+d detents -> %s\n", detents, label);
    }
  }

  void shortPressEncoder() {
    if (sleeping) {
      wake("BUTTON");
      return;
    }

    if (menuActive) {
      selectMenuItem();
      return;
    }

    if (calState == CalWaitHeel) {
      settings.heelRaw = currentRaw;
      calState = CalWaitToe;
      configureProcessor();
      markDirty();
      printf("ENCODER: short press captured heel raw=%u\n", settings.heelRaw);
      return;
    }

    if (calState == CalWaitToe) {
      settings.toeRaw = currentRaw;
      calState = CalIdle;
      configureProcessor();
      markDirty();
      save();
      printf("ENCODER: short press captured toe raw=%u and saved\n", settings.toeRaw);
      return;
    }

    if (tuneMode) {
      tuneMode = false;
      save();
      printf("ENCODER: short press -> tune off saved\n");
      return;
    }

    if (outputFocusIsLfoOut()) {
      bool tapped = applyTapTempo();
      lfoOutPhase[currentLfoOutIndex()] = 0.0f;
      lfoOutCycle[currentLfoOutIndex()] = 0;
      printf("ENCODER: short press -> %s %s\n",
             outputFocusLabel(settings.outputFocus),
             tapped ? "tap" : "sync");
    } else if (outputModeIsLfo()) {
      bool tapped = applyTapTempo();
      lfoPhase = 0.0f;
      lfoCycle = 0;
      printf("ENCODER: short press -> LFO %s\n", tapped ? "tap" : "sync");
    } else {
      settings.semitones = signedPicoInterval(0, settings.bendDirection);
      markDirty();
      printf("ENCODER: short press -> unison interval\n");
    }
  }

  void doubleClickEncoder() {
    if (sleeping || calState != CalIdle || tuneMode) {
      return;
    }
    commandFocus(static_cast<uint8_t>((settings.outputFocus + 1) % OutputFocusCount));
    save();
    printf("ENCODER: double click -> focus %s\n", outputFocusLabel(settings.outputFocus));
  }

  void holdEncoder(uint32_t holdMs) {
    if (sleeping) {
      wake("BUTTON");
      return;
    }

    if (holdMs >= 2000 && calState == CalIdle) {
      menuActive = !menuActive;
      menuIndex = MenuOut;
      menuEditField = MenuEditNone;
      printf("ENCODER: hold %ums -> menu %s\n", holdMs, menuActive ? "open" : "closed");
      return;
    }

    shortPressEncoder();
  }

  void advanceMs(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
      tick1ms();
    }
  }

  Snapshot snapshot() const {
    Snapshot s;
    s.outputMicrovolts = outputMicrovolts;
    s.dacCode = dacCode;
    for (uint8_t i = 0; i < kLfoOutCount; ++i) {
      s.lfoOutMicrovolts[i] = lfoOutOutputMicrovolts[i];
      s.lfoOutDacCode[i] = lfoOutDacCode[i];
    }
    s.clockDacCode = clockDacCode;
    s.raw = pedalState.raw;
    s.pedal = pedalState.curved;
    return s;
  }

  bool isSleeping() const {
    return sleeping;
  }

  bool isDirty() const {
    return settingsDirty;
  }

  bool isMenuActive() const {
    return menuActive;
  }

  bool isMenuEditing() const {
    return menuEditField != MenuEditNone;
  }

  float liveLfoRateHz() const {
    return currentLfoRateHz();
  }

  float lfoOutLiveRateHz(uint8_t index) const {
    return lfoOutRateHz(index);
  }

  bool isLfoOutFocus() const {
    return outputFocusIsLfoOut();
  }

  Settings currentSettings() const {
    return settings;
  }

  void printStatus(const char* title) const {
    char label[8];
    signedIntervalLabel(label, sizeof(label));

    printf("\n== %s ==\n", title);
    printf("SERIAL: raw=%u norm=%.4f filt=%.4f interval=%s semis=%d focus=%s mode=%s dir=%s noBendMv=%.3f expMv=%.3f expCode=%u tune=%s stepMv=%u responseCents=%u curve=%s wave=%s rateHz=%.3f maxRateHz=%.3f depth=%u%% cv=%s lfo1Mv=%.3f lfo1Code=%u lfo1Mode=%s lfo1Wave=%s lfo1RateHz=%.3f lfo1Depth=%u%% lfo1Pol=%s lfo2Mv=%.3f lfo2Code=%u lfo2Mode=%s lfo2Wave=%s lfo2RateHz=%.3f lfo2Depth=%u%% lfo2Pol=%s dac=%s oled=%s cal=%s menu=%s sleep=%s heel=%u toe=%u invert=%s dirty=%s\n",
           pedalState.raw,
           pedalState.normalized,
           pedalState.filtered,
           label,
           settings.semitones,
           outputFocusLabel(settings.outputFocus),
           picoOutputModeName(settings.outputMode),
           directionLabel(),
           noBendMicrovolts() / 1000.0,
           outputMicrovolts / 1000.0,
           dacCode,
           tuneMode ? "on" : "off",
           tuneStepMv,
           settings.responseCents,
           picoCurveName(settings.curveMode),
           picoLfoWaveName(settings.lfoWave),
           currentLfoRateHz(),
           configuredLfoMaxRateHz(),
           settings.lfoDepthPercent,
           cvOverride ? "on" : "off",
           lfoOutOutputMicrovolts[0] / 1000.0,
           lfoOutDacCode[0],
           picoOutputModeName(settings.lfoOutMode[0]),
           picoLfoWaveName(settings.lfoOutWave[0]),
           lfoOutRateHz(0),
           settings.lfoOutDepthPercent[0],
           settings.lfoOutPolarity[0] == kPicoBendDown ? "DN" : "UP",
           lfoOutOutputMicrovolts[1] / 1000.0,
           lfoOutDacCode[1],
           picoOutputModeName(settings.lfoOutMode[1]),
           picoLfoWaveName(settings.lfoOutWave[1]),
           lfoOutRateHz(1),
           settings.lfoOutDepthPercent[1],
           settings.lfoOutPolarity[1] == kPicoBendDown ? "DN" : "UP",
           dacPresent ? "ready" : "missing",
           oledPresent ? "ready" : "missing",
           calStateName(),
           menuActive ? "on" : "off",
           sleeping ? "on" : "off",
           settings.heelRaw,
           settings.toeRaw,
           settings.invert ? "on" : "off",
           settingsDirty ? "yes" : "no");
    printOled();
  }

 private:
  Settings settings;
  PedalProcessor processor;
  PedalState pedalState;
  CalibrationState calState = CalIdle;
  uint16_t currentRaw = 0;
  uint16_t lastActivityRaw = 0;
  int32_t outputMicrovolts = kDefaultCenterMicrovolts;
  int32_t lfoOutOutputMicrovolts[kLfoOutCount] = {0, 0};
  uint16_t dacCode = 0;
  uint16_t lfoOutDacCode[kLfoOutCount] = {0, 0};
  uint32_t nowMs = 0;
  uint32_t lastActivityMs = 0;
  uint32_t lastSettingsChangeMs = 0;
  bool dacPresent = true;
  bool oledPresent = true;
  bool sleeping = false;
  bool settingsDirty = false;
  bool tuneMode = false;
  bool cvOverride = false;
  bool menuActive = false;
  MenuItem menuIndex = MenuOut;
  MenuEditField menuEditField = MenuEditNone;
  uint8_t menuEditValue = 0;
  float lfoPhase = 0.0f;
  uint32_t lfoCycle = 0;
  float lfoOutPhase[kLfoOutCount] = {0.0f, 0.0f};
  uint32_t lfoOutCycle[kLfoOutCount] = {0, 0};
  uint16_t clockDacCode = 0;
  uint32_t lastTapMs = 0;
  uint8_t lastTapFocus = 0xff;
  uint16_t cvOverrideMv = 0;
  uint16_t tuneStepMv = kDefaultTuneStepMv;

  void fillLinearToeMap() {
    for (int semitone = kPicoMinSemitones; semitone <= kPicoMaxSemitones; ++semitone) {
      int32_t toeUv = computePicoUnipolarToeMicrovolts(
          semitone,
          static_cast<int32_t>(settings.rangeMv) * 1000L);
      settings.toeMapMv[picoSemitoneMapIndex(semitone)] =
          static_cast<uint16_t>(clampValue<int32_t>((toeUv + 500L) / 1000L,
                                                    0,
                                                    kMaxOutputMv));
    }
    settings.toeMapMv[picoSemitoneMapIndex(0)] = 0;
  }

  void fillResponseToeMap() {
    for (int semitone = kPicoMinSemitones; semitone <= kPicoMaxSemitones; ++semitone) {
      int32_t toeUv = computePicoResponseToeMicrovolts(
          semitone,
          settings.responseCents);
      settings.toeMapMv[picoSemitoneMapIndex(semitone)] =
          static_cast<uint16_t>(clampValue<int32_t>((toeUv + 500L) / 1000L,
                                                    0,
                                                    kMaxOutputMv));
    }
    settings.toeMapMv[picoSemitoneMapIndex(0)] = 0;
  }

  void fillToeMapFromCurrentFit() {
    if (settings.responseCents > 0) {
      fillResponseToeMap();
    } else {
      fillLinearToeMap();
    }
  }

  void configureProcessor() {
    PedalCalibration calibration;
    calibration.heelRaw = settings.heelRaw;
    calibration.toeRaw = settings.toeRaw;
    calibration.invert = settings.invert;
    calibration.snapLow = 0.005f;
    calibration.snapHigh = 0.995f;
    calibration.deadband = 0.0005f;

    PedalFilterSettings filterSettings;
    filterSettings.enabled = true;
    filterSettings.minCutoffHz = 3.5f;
    filterSettings.beta = 0.12f;
    filterSettings.derivativeCutoffHz = 1.0f;

    processor.configure(calibration, filterSettings, settings.curveMode);
    processor.reset();
  }

  void tick1ms() {
    ++nowMs;
    pedalState = processor.process(currentRaw, 1000.0f);

    if (abs(static_cast<int>(currentRaw) - static_cast<int>(lastActivityRaw)) >
        kPedalActivityRawDelta) {
      lastActivityRaw = currentRaw;
      lastActivityMs = nowMs;
      if (sleeping) {
        wake("PEDAL");
      }
    }

    if (cvOverride) {
      outputMicrovolts = static_cast<int32_t>(cvOverrideMv) * 1000L;
    } else if (outputModeIsLfo()) {
      lfoPhase += currentLfoRateHz() * 0.001f;
      float wrapped = floorf(lfoPhase);
      lfoCycle += static_cast<uint32_t>(wrapped);
      lfoPhase -= wrapped;
      float value = computePicoLfoWaveValue(lfoPhase,
                                            settings.lfoWave,
                                            lfoCycle,
                                            kLfoSeedExp,
                                            settings.lfoPulseWidth);
      if (settings.bendDirection == kPicoBendDown) {
        value = 1.0f - value;
      }
      value = attenuatePicoLfoWaveValue(value, settings.lfoDepthPercent);
      outputMicrovolts =
          static_cast<int32_t>(lroundf(value * static_cast<float>(kPicoDacFullScaleMicrovolts)));
    } else {
      outputMicrovolts =
          computePicoMappedOutputMicrovolts(
              pedalState.curved,
              noBendMicrovolts(),
              toeMicrovoltsForCurrentInterval());
    }
    dacCode = microvoltsToMcp4728Code(outputMicrovolts);

    for (uint8_t i = 0; i < kLfoOutCount; ++i) {
      if (i == 1 && settings.lfo2Link != kPicoLfoLinkOff) {
        computePicoLinkedPhase(lfoOutPhase[0],
                               lfoOutCycle[0],
                               settings.lfo2Link,
                               settings.lfo2PhaseOffset,
                               &lfoOutPhase[1],
                               &lfoOutCycle[1]);
      } else {
        lfoOutPhase[i] += lfoOutRateHz(i) * 0.001f;
        float wrapped = floorf(lfoOutPhase[i]);
        lfoOutCycle[i] += static_cast<uint32_t>(wrapped);
        lfoOutPhase[i] -= wrapped;
      }
      float value = computePicoLfoWaveValue(lfoOutPhase[i],
                                            settings.lfoOutWave[i],
                                            lfoOutCycle[i],
                                            kLfoSeedOut[i],
                                            settings.lfoOutPulseWidth[i]);
      if (settings.lfoOutPolarity[i] == kPicoBendDown) {
        value = 1.0f - value;
      }
      value = attenuatePicoLfoWaveValue(value, settings.lfoOutDepthPercent[i]);
      value = offsetPicoLfoWaveValue(value, settings.lfoOutOffsetPercent[i]);
      lfoOutOutputMicrovolts[i] =
          static_cast<int32_t>(lroundf(value * static_cast<float>(kPicoDacFullScaleMicrovolts)));
      lfoOutDacCode[i] = microvoltsToMcp4728Code(lfoOutOutputMicrovolts[i]);
    }

    if (settings.clockSource == kPicoClockOff) {
      clockDacCode = 0;
    } else {
      uint8_t source = settings.clockSource == kPicoClockLfo2 ? 1 : 0;
      clockDacCode = lfoOutPhase[source] < 0.5f ? kMcp4728MaxCode : 0;
    }

    if (!sleeping && calState == CalIdle && settingsDirty &&
        nowMs - lastSettingsChangeMs >= kAutosaveMs) {
      save();
    }

    if (kIdleSleepEnabled &&
        !sleeping && !tuneMode && !cvOverride && nowMs - lastActivityMs >= kIdleSleepMs) {
      sleep("IDLE");
    }
  }

  void markDirty() {
    settingsDirty = true;
    lastSettingsChangeMs = nowMs;
  }

  void syncFocusedLfo() {
    if (outputFocusIsLfoOut()) {
      lfoOutPhase[currentLfoOutIndex()] = 0.0f;
      lfoOutCycle[currentLfoOutIndex()] = 0;
    } else {
      lfoPhase = 0.0f;
      lfoCycle = 0;
    }
  }

  bool applyTapTempo() {
    uint32_t delta = nowMs - lastTapMs;
    bool tapped = lastTapMs != 0 && lastTapFocus == settings.outputFocus &&
                  delta >= kTapMinMs && delta <= kTapMaxMs;
    lastTapMs = nowMs;
    lastTapFocus = settings.outputFocus;
    if (!tapped) {
      return false;
    }
    if (settings.outputFocus == OutputFocusLfo2 && settings.lfo2Link != kPicoLfoLinkOff) {
      printf("SERIAL: LFO2 linked to LFO1; tap ignored\n");
      return false;
    }
    float hz = 1000.0f / static_cast<float>(delta);
    if (outputFocusIsLfoOut()) {
      uint8_t index = currentLfoOutIndex();
      uint8_t mode = settings.lfoOutMode[index];
      uint8_t step = nearestPicoLfoRateStep(mode, hz);
      if (mode == kPicoOutputLfoFm) {
        settings.lfoOutFmRate[index] = step;
      } else {
        settings.lfoOutLoRate[index] = step;
      }
    } else {
      uint8_t step = nearestPicoLfoRateStep(settings.outputMode, hz);
      if (settings.outputMode == kPicoOutputLfoFm) {
        settings.lfoFmRate = step;
      } else {
        settings.lfoLoRate = step;
      }
    }
    markDirty();
    return true;
  }

  void save() {
    settingsDirty = false;
    printf("SERIAL: OK saved settings\n");
  }

  void sleep(const char* reason) {
    sleeping = true;
    printf("SERIAL: SLEEP %s\n", reason);
  }

  void wake(const char* reason) {
    sleeping = false;
    lastActivityMs = nowMs;
    printf("SERIAL: WAKE %s\n", reason);
  }

  void signedIntervalLabel(char* out, size_t outSize) const {
    int clamped = clampPicoSemitones(settings.semitones);
    const char* absLabel = picoAbsoluteIntervalLabel(static_cast<uint8_t>(abs(clamped)));
    if (clamped > 0) {
      snprintf(out, outSize, "+%s", absLabel);
    } else if (clamped < 0) {
      snprintf(out, outSize, "-%s", absLabel);
    } else {
      snprintf(out, outSize, "1");
    }
  }

  const char* directionLabel() const {
    return settings.bendDirection == kPicoBendDown ? "DOWN" : "UP";
  }

  bool outputFocusIsExp() const {
    return settings.outputFocus == OutputFocusExp;
  }

  bool outputFocusIsLfoOut() const {
    return !outputFocusIsExp();
  }

  uint8_t currentLfoOutIndex() const {
    return settings.outputFocus == OutputFocusLfo2 ? 1 : 0;
  }

  const char* outputFocusLabel(uint8_t focus) const {
    switch (focus) {
      case OutputFocusLfo1:
        return "LFO1";
      case OutputFocusLfo2:
        return "LFO2";
      case OutputFocusExp:
      default:
        return "EXP";
    }
  }

  bool outputModeIsLfo() const {
    return settings.outputMode == kPicoOutputLfoLo || settings.outputMode == kPicoOutputLfoFm;
  }

  uint8_t currentLfoRateStep() const {
    return settings.outputMode == kPicoOutputLfoFm ? settings.lfoFmRate : settings.lfoLoRate;
  }

  uint8_t lfoOutRateStep(uint8_t index) const {
    uint8_t safeIndex = index < kLfoOutCount ? index : 0;
    return settings.lfoOutMode[safeIndex] == kPicoOutputLfoFm
               ? settings.lfoOutFmRate[safeIndex]
               : settings.lfoOutLoRate[safeIndex];
  }

  uint8_t currentFocusedLfoRateStep() const {
    if (outputFocusIsLfoOut()) {
      return lfoOutRateStep(currentLfoOutIndex());
    }
    return currentLfoRateStep();
  }

  float currentLfoRateHz() const {
    return computePicoLfoRateHzForPedal(settings.outputMode, pedalState.filtered, currentLfoRateStep());
  }

  float configuredLfoMaxRateHz() const {
    return computePicoLfoRateHz(settings.outputMode, currentLfoRateStep());
  }

  float lfoOutRateHz(uint8_t index) const {
    uint8_t safeIndex = index < kLfoOutCount ? index : 0;
    return computePicoLfoRateHz(settings.lfoOutMode[safeIndex], lfoOutRateStep(safeIndex));
  }

  float currentFocusedLfoRateHz() const {
    if (outputFocusIsLfoOut()) {
      return lfoOutRateHz(currentLfoOutIndex());
    }
    return configuredLfoMaxRateHz();
  }

  uint8_t currentFocusedLfoDepthPercent() const {
    if (outputFocusIsLfoOut()) {
      return settings.lfoOutDepthPercent[currentLfoOutIndex()];
    }
    return settings.lfoDepthPercent;
  }

  uint8_t currentFocusedLfoWave() const {
    if (outputFocusIsLfoOut()) {
      return settings.lfoOutWave[currentLfoOutIndex()];
    }
    return settings.lfoWave;
  }

  uint8_t currentFocusedOutputMode() const {
    if (outputFocusIsLfoOut()) {
      return settings.lfoOutMode[currentLfoOutIndex()];
    }
    return settings.outputMode;
  }

  int32_t noBendMicrovolts() const {
    return picoUnipolarNoBendMicrovolts(settings.bendDirection);
  }

  int32_t toeMicrovoltsForCurrentInterval() const {
    if (settings.semitones == 0) {
      return noBendMicrovolts();
    }
    return static_cast<int32_t>(settings.toeMapMv[picoSemitoneMapIndex(settings.semitones)]) *
           1000L;
  }

  const char* calStateName() const {
    switch (calState) {
      case CalWaitHeel:
        return "heel";
      case CalWaitToe:
        return "toe";
      case CalIdle:
      default:
        return "idle";
    }
  }

  const char* menuItemTitle(MenuItem item) const {
    switch (item) {
      case MenuOut:
        return "OUT";
      case MenuMode:
        return "MODE";
      case MenuWave:
        return "WAVE";
      case MenuDepth:
        return "DEPTH";
      case MenuPw:
        return "PW";
      case MenuOfs:
        return "OFS";
      case MenuLink:
        return "LINK";
      case MenuPhs:
        return "PHS";
      case MenuClk:
        return "CLK";
      case MenuCurve:
        return "CURVE";
      case MenuCal:
        return "CAL";
      case MenuDir:
        return outputFocusIsLfoOut() || outputModeIsLfo() ? "POL" : "DIR";
      case MenuDone:
        return "DONE";
      default:
        return "MENU";
    }
  }

  uint8_t currentFocusedLfoPulseWidth() const {
    if (outputFocusIsLfoOut()) {
      return settings.lfoOutPulseWidth[currentLfoOutIndex()];
    }
    return settings.lfoPulseWidth;
  }

  bool menuItemVisible(MenuItem item) const {
    bool focusedLfoSettingsApply = outputFocusIsLfoOut() || outputModeIsLfo();
    if ((item == MenuWave || item == MenuDepth) && !focusedLfoSettingsApply) {
      return false;
    }
    if (item == MenuPw &&
        (!focusedLfoSettingsApply || currentFocusedLfoWave() != kPicoLfoPulse)) {
      return false;
    }
    if (item == MenuOfs && !outputFocusIsLfoOut()) {
      return false;
    }
    if ((item == MenuLink || item == MenuPhs) && settings.outputFocus != OutputFocusLfo2) {
      return false;
    }
    if (item == MenuPhs && settings.lfo2Link == kPicoLfoLinkOff) {
      return false;
    }
    if ((item == MenuCurve || item == MenuCal) && outputFocusIsLfoOut()) {
      return false;
    }
    return item >= MenuOut && item < MenuCount;
  }

  uint8_t wrapMenuChoice(int value, int count) const {
    if (count <= 0) {
      return 0;
    }
    while (value < 0) {
      value += count;
    }
    return static_cast<uint8_t>(value % count);
  }

  const char* menuEditTitle(MenuEditField field) const {
    switch (field) {
      case MenuEditOut:
        return "SET OUT";
      case MenuEditMode:
        return "SET MODE";
      case MenuEditWave:
        return "SET WAVE";
      case MenuEditDepth:
        return "SET DEP";
      case MenuEditPw:
        return "SET PW";
      case MenuEditOfs:
        return "SET OFS";
      case MenuEditLink:
        return "SET LINK";
      case MenuEditPhs:
        return "SET PHS";
      case MenuEditClk:
        return "SET CLK";
      case MenuEditCurve:
        return "SET CURVE";
      default:
        return "SET";
    }
  }

  uint8_t menuEditValueCount(MenuEditField field) const {
    switch (field) {
      case MenuEditOut:
        return OutputFocusCount;
      case MenuEditMode:
        return outputFocusIsLfoOut() ? 2 : kPicoOutputModeCount;
      case MenuEditWave:
        return kPicoLfoWaveCount;
      case MenuEditDepth:
        return kPicoLfoDepthStepCount;
      case MenuEditPw:
        return kPicoLfoPulseWidthStepCount;
      case MenuEditOfs:
        return kPicoLfoOffsetStepCount;
      case MenuEditLink:
        return kPicoLfoLinkCount;
      case MenuEditPhs:
        return kPicoLfoPhaseOffsetCount;
      case MenuEditClk:
        return kPicoClockSourceCount;
      case MenuEditCurve:
        return kPicoCurveCount;
      default:
        return 1;
    }
  }

  uint8_t currentMenuEditValue(MenuEditField field) const {
    switch (field) {
      case MenuEditOut:
        return settings.outputFocus;
      case MenuEditMode:
        return outputFocusIsLfoOut()
                   ? (settings.lfoOutMode[currentLfoOutIndex()] == kPicoOutputLfoFm ? 1 : 0)
                   : settings.outputMode;
      case MenuEditWave:
        return outputFocusIsLfoOut() ? settings.lfoOutWave[currentLfoOutIndex()]
                                     : settings.lfoWave;
      case MenuEditDepth:
        return picoLfoDepthIndexFromPercent(currentFocusedLfoDepthPercent());
      case MenuEditPw:
        return picoLfoPulseWidthIndexFromPercent(currentFocusedLfoPulseWidth());
      case MenuEditOfs:
        return picoLfoOffsetIndexFromPercent(
            settings.lfoOutOffsetPercent[currentLfoOutIndex()]);
      case MenuEditLink:
        return settings.lfo2Link;
      case MenuEditPhs:
        return settings.lfo2PhaseOffset;
      case MenuEditClk:
        return settings.clockSource;
      case MenuEditCurve:
        return settings.curveMode;
      default:
        return 0;
    }
  }

  const char* menuEditValueLabel(MenuEditField field, uint8_t value) const {
    switch (field) {
      case MenuEditOut:
        return outputFocusLabel(value);
      case MenuEditMode:
        return outputFocusIsLfoOut() ? picoOutputModeDisplayLabel(value == 0 ? kPicoOutputLfoLo
                                                                              : kPicoOutputLfoFm)
                                     : picoOutputModeDisplayLabel(value);
      case MenuEditWave:
        return picoLfoWaveDisplayLabel(value);
      case MenuEditDepth: {
        static char depth[8];
        snprintf(depth, sizeof(depth), "%u%%", picoLfoDepthPercentFromIndex(value));
        return depth;
      }
      case MenuEditPw: {
        static char pw[8];
        snprintf(pw, sizeof(pw), "%u%%", picoLfoPulseWidthPercentFromIndex(value));
        return pw;
      }
      case MenuEditOfs: {
        static char ofs[8];
        snprintf(ofs, sizeof(ofs), "%+d%%",
                 static_cast<int>(picoLfoOffsetPercentFromIndex(value)));
        return ofs;
      }
      case MenuEditLink:
        return picoLfoLinkName(value);
      case MenuEditPhs: {
        static char phs[8];
        snprintf(phs, sizeof(phs), "%u", picoLfoPhaseOffsetDegrees(value));
        return phs;
      }
      case MenuEditClk:
        return picoClockSourceDisplayLabel(value);
      case MenuEditCurve:
        return picoCurveDisplayLabel(value);
      default:
        return "";
    }
  }

  void beginMenuEdit(MenuEditField field) {
    menuEditField = field;
    menuEditValue = currentMenuEditValue(field);
    printf("ENCODER: menu edit %s %s\n",
           menuEditTitle(menuEditField),
           menuEditValueLabel(menuEditField, menuEditValue));
  }

  void commitMenuEdit() {
    MenuEditField field = menuEditField;
    uint8_t value = menuEditValue;
    switch (field) {
      case MenuEditOut:
        commandFocus(value);
        save();
        break;
      case MenuEditMode:
        commandMode(outputFocusIsLfoOut() ? (value == 0 ? kPicoOutputLfoLo : kPicoOutputLfoFm)
                                          : value);
        save();
        break;
      case MenuEditWave:
        commandWave(value);
        save();
        break;
      case MenuEditDepth:
        commandDepth(picoLfoDepthPercentFromIndex(value));
        save();
        break;
      case MenuEditPw:
        commandPulseWidth(picoLfoPulseWidthPercentFromIndex(value));
        save();
        break;
      case MenuEditOfs:
        commandOffset(picoLfoOffsetPercentFromIndex(value));
        save();
        break;
      case MenuEditLink:
        commandLink(value);
        save();
        break;
      case MenuEditPhs:
        commandLinkPhase(value);
        save();
        break;
      case MenuEditClk:
        commandClock(value);
        save();
        break;
      case MenuEditCurve:
        commandCurve(value);
        save();
        break;
      default:
        break;
    }
    menuEditField = MenuEditNone;
  }

  void selectMenuItem() {
    if (menuEditField != MenuEditNone) {
      commitMenuEdit();
      return;
    }

    switch (menuIndex) {
      case MenuOut:
        beginMenuEdit(MenuEditOut);
        break;
      case MenuMode:
        beginMenuEdit(MenuEditMode);
        break;
      case MenuWave:
        beginMenuEdit(MenuEditWave);
        break;
      case MenuDepth:
        beginMenuEdit(MenuEditDepth);
        break;
      case MenuPw:
        beginMenuEdit(MenuEditPw);
        break;
      case MenuOfs:
        beginMenuEdit(MenuEditOfs);
        break;
      case MenuLink:
        beginMenuEdit(MenuEditLink);
        break;
      case MenuPhs:
        beginMenuEdit(MenuEditPhs);
        break;
      case MenuClk:
        beginMenuEdit(MenuEditClk);
        break;
      case MenuCurve:
        beginMenuEdit(MenuEditCurve);
        break;
      case MenuCal:
        menuActive = false;
        menuEditField = MenuEditNone;
        calState = CalWaitHeel;
        printf("ENCODER: menu cal -> CAL HEEL\n");
        break;
      case MenuDir:
        if (outputFocusIsLfoOut()) {
          uint8_t index = currentLfoOutIndex();
          settings.lfoOutPolarity[index] =
              settings.lfoOutPolarity[index] == kPicoBendDown ? kPicoBendUp : kPicoBendDown;
          markDirty();
        } else {
          commandDirection(settings.bendDirection == kPicoBendDown ? kPicoBendUp
                                                                   : kPicoBendDown);
        }
        save();
        printf("ENCODER: menu dir/pol -> %s\n",
               outputFocusIsLfoOut()
                   ? (settings.lfoOutPolarity[currentLfoOutIndex()] == kPicoBendDown ? "DN"
                                                                                     : "UP")
                   : directionLabel());
        break;
      case MenuDone:
      default:
        menuActive = false;
        menuEditField = MenuEditNone;
        printf("ENCODER: menu exit\n");
        break;
    }
  }

  void printOled() const {
    if (!oledPresent) {
      printf("OLED: [not detected]\n");
      return;
    }
    if (sleeping) {
      printf("OLED: [off]\n");
      return;
    }
    if (menuActive) {
      printf("OLED:\n");
      if (menuEditField != MenuEditNone) {
        printf("  %-8s %s\n",
               menuEditTitle(menuEditField),
               menuEditValueLabel(menuEditField, menuEditValue));
        printf("          TURN PRESS OK\n");
      } else {
        char depthLabel[8];
        snprintf(depthLabel, sizeof(depthLabel), "%u%%", currentFocusedLfoDepthPercent());
        printf("  MENU     %s %s\n",
               menuItemTitle(menuIndex),
               menuIndex == MenuOut    ? outputFocusLabel(settings.outputFocus)
               : menuIndex == MenuMode ? picoOutputModeDisplayLabel(currentFocusedOutputMode())
               : menuIndex == MenuWave ? picoLfoWaveDisplayLabel(currentFocusedLfoWave())
               : menuIndex == MenuDepth ? depthLabel
               : menuIndex == MenuCurve ? picoCurveDisplayLabel(settings.curveMode)
               : menuIndex == MenuDir   ? (outputFocusIsLfoOut()
                                               ? (settings.lfoOutPolarity[currentLfoOutIndex()] ==
                                                          kPicoBendDown
                                                      ? "DN"
                                                      : "UP")
                                               : directionLabel())
                                        : "");
        printf("          TURN ITEM PRESS OK\n");
      }
      return;
    }

    char label[8];
    if (outputFocusIsLfoOut()) {
      snprintf(label, sizeof(label), "%s", outputFocusLabel(settings.outputFocus));
    } else if (cvOverride) {
      snprintf(label, sizeof(label), "CV");
    } else if (outputModeIsLfo()) {
      snprintf(label, sizeof(label), "%s", picoOutputModeDisplayLabel(settings.outputMode));
    } else {
      signedIntervalLabel(label, sizeof(label));
    }

    printf("OLED:\n");
    if (outputFocusIsLfoOut()) {
      printf("  %-5s   SPD %.3fHZ\n", label, lfoOutRateHz(currentLfoOutIndex()));
    } else if (outputModeIsLfo() && !cvOverride) {
      printf("  %-5s   SPD PED\n", label);
    } else {
      int32_t toeUv =
          cvOverride
              ? static_cast<int32_t>(cvOverrideMv) * 1000L
              : toeMicrovoltsForCurrentInterval();
      printf("  %-5s   %s %4ldMV\n",
             label,
             cvOverride ? "SET" : "TOE",
             static_cast<long>(toeUv / 1000));
    }
    if (outputFocusIsLfoOut()) {
      printf("          DEP %u%%\n", currentFocusedLfoDepthPercent());
    } else if (cvOverride) {
      printf("          FIXED\n");
    } else if (outputModeIsLfo()) {
      printf("          DEP %u%%\n", settings.lfoDepthPercent);
    } else if (tuneMode) {
      printf("          STEP %uMV\n", tuneStepMv);
    } else if (settings.responseCents > 0) {
      printf("          RSP %uC\n", settings.responseCents);
    } else {
      printf("          RNG %uMV\n", settings.rangeMv);
    }
    if (outputFocusIsLfoOut()) {
      printf("          %s %s\n",
             picoLfoWaveDisplayLabel(currentFocusedLfoWave()),
             settings.lfoOutPolarity[currentLfoOutIndex()] == kPicoBendDown ? "DN" : "UP");
    } else if (outputModeIsLfo()) {
      printf("          %s %s\n",
             picoLfoWaveDisplayLabel(settings.lfoWave),
             settings.bendDirection == kPicoBendDown ? "DN" : "UP");
    } else {
      printf("          CRV %s\n", picoCurveDisplayLabel(settings.curveMode));
    }
    printf("  %s\n",
           dacPresent
               ? (outputFocusIsLfoOut() ? outputFocusLabel(settings.outputFocus)
                                         : (cvOverride ? "CV HOLD"
                                                       : (tuneMode ? "TUNE" : directionLabel())))
               : "DAC FAIL");
  }
};

void expectNearMv(const char* label, const Snapshot& snapshot, float expectedMv, float toleranceMv) {
  float actualMv = snapshot.outputMicrovolts / 1000.0f;
  if (fabsf(actualMv - expectedMv) > toleranceMv) {
    fprintf(stderr,
            "ASSERT FAIL %s: expected %.3fmV +/- %.3fmV, got %.3fmV\n",
            label,
            expectedMv,
            toleranceMv,
            actualMv);
    exit(1);
  }
}

void runScriptedSimulation() {
  PicoConsoleSim sim;
  sim.boot();

  expectNearMv("boot heel +6", sim.snapshot(), 0.0f, 2.0f);

  sim.commandInterval(12);
  sim.setPedalRaw(0);
  sim.printStatus("+6 heel after over-range clamp");
  expectNearMv("+6 heel", sim.snapshot(), 0.0f, 2.0f);

  sim.setPedalRaw(4095);
  sim.printStatus("+6 toe after over-range clamp");
  expectNearMv("+6 toe", sim.snapshot(), 3214.0f, 2.0f);

  sim.commandInterval(-12);
  sim.setPedalRaw(0);
  sim.printStatus("-6 heel after over-range clamp");
  expectNearMv("-6 heel", sim.snapshot(), 3300.0f, 2.0f);

  sim.setPedalRaw(4095);
  sim.printStatus("-6 toe after over-range clamp");
  expectNearMv("-6 toe", sim.snapshot(), 86.0f, 2.0f);

  sim.commandDirection(kPicoBendUp);
  assert(sim.currentSettings().bendDirection == kPicoBendUp);
  assert(sim.currentSettings().semitones == kPicoMaxSemitones);
  sim.setPedalRaw(0);
  expectNearMv("direction up heel", sim.snapshot(), 0.0f, 2.0f);

  sim.commandInterval(4);
  sim.setPedalRaw(4095);
  sim.printStatus("response calibrated major third");
  expectNearMv("response calibrated major third", sim.snapshot(), 1429.0f, 2.0f);

  sim.commandResponse(3960);
  sim.setPedalRaw(4095);
  expectNearMv("response 3960 standard 1v/oct major third", sim.snapshot(), 333.0f, 2.0f);
  sim.commandInterval(12);
  sim.setPedalRaw(4095);
  expectNearMv("response 3960 standard 1v/oct sixth", sim.snapshot(), 750.0f, 2.0f);

  sim.commandResponse(924);
  sim.commandInterval(4);
  sim.setPedalRaw(4095);
  expectNearMv("response 924 restored", sim.snapshot(), 1429.0f, 2.0f);

  sim.commandInterval(9);
  sim.commandCurve(kPicoCurveLinear);
  sim.setPedalRaw(2048);
  float linearMidMv = sim.snapshot().outputMicrovolts / 1000.0f;
  sim.commandCurve(kPicoCurveEaseOut);
  sim.setPedalRaw(2048);
  float easeOutMidMv = sim.snapshot().outputMicrovolts / 1000.0f;
  sim.commandCurve(kPicoCurveSquare);
  sim.setPedalRaw(2048);
  float squareMidMv = sim.snapshot().outputMicrovolts / 1000.0f;
  assert(easeOutMidMv > linearMidMv + 250.0f);
  assert(squareMidMv < linearMidMv - 250.0f);
  sim.commandCurve(kPicoCurveLinear);
  sim.setPedalRaw(4095);
  sim.printStatus("curve response comparison restored to linear");

  sim.commandMap(4, 3000);
  sim.commandInterval(4);
  sim.setPedalRaw(4095);
  sim.printStatus("custom mapped major third");
  expectNearMv("mapped major third toe", sim.snapshot(), 3000.0f, 2.0f);

  sim.commandCv(1234);
  sim.setPedalRaw(0);
  sim.printStatus("fixed cv override");
  expectNearMv("fixed cv override", sim.snapshot(), 1234.0f, 2.0f);
  sim.commandCvOff();
  sim.setPedalRaw(4095);
  expectNearMv("cv off resumes pedal", sim.snapshot(), 3000.0f, 2.0f);

  sim.commandInterval(3);
  sim.commandMap(3, 1875);
  sim.commandTuneStep(25);
  sim.commandTune(true);
  sim.turnEncoder(4);
  sim.setPedalRaw(4095);
  sim.printStatus("tune mode nudged b3");
  expectNearMv("tune mode nudged b3", sim.snapshot(), 1975.0f, 2.0f);
  sim.shortPressEncoder();
  assert(!sim.isDirty());

  sim.commandInterval(-3);
  sim.commandMap(-3, 2475);
  sim.commandTuneStep(25);
  sim.commandTune(true);
  sim.turnEncoder(2);
  sim.setPedalRaw(4095);
  sim.printStatus("down tune mode increases interval");
  expectNearMv("down tune mode increases interval", sim.snapshot(), 2425.0f, 2.0f);
  sim.shortPressEncoder();
  assert(!sim.isDirty());
  sim.commandDirection(kPicoBendUp);

  sim.holdEncoder(2000);
  assert(sim.isMenuActive());
  assert(!sim.isMenuEditing());
  sim.turnEncoder(1);
  sim.shortPressEncoder();
  assert(sim.isMenuActive());
  assert(sim.isMenuEditing());
  sim.turnEncoder(1);
  sim.shortPressEncoder();
  assert(sim.isMenuActive());
  assert(!sim.isMenuEditing());
  assert(sim.currentSettings().outputMode == kPicoOutputLfoLo);
  sim.turnEncoder(1);
  sim.shortPressEncoder();
  assert(sim.isMenuEditing());
  sim.turnEncoder(1);
  sim.shortPressEncoder();
  assert(!sim.isMenuEditing());
  assert(sim.currentSettings().lfoWave == kPicoLfoTriangle);
  sim.turnEncoder(1);
  sim.shortPressEncoder();
  assert(sim.isMenuEditing());
  sim.turnEncoder(-5);
  sim.shortPressEncoder();
  assert(!sim.isMenuEditing());
  assert(sim.currentSettings().lfoDepthPercent == 75);
  sim.turnEncoder(4);
  sim.shortPressEncoder();
  assert(sim.currentSettings().bendDirection == kPicoBendDown);
  sim.turnEncoder(1);
  sim.shortPressEncoder();
  assert(!sim.isMenuActive());
  sim.commandDirection(kPicoBendUp);
  sim.commandFocus(OutputFocusExp);
  sim.commandMode(kPicoOutputPedal);
  sim.commandCurve(kPicoCurveLinear);

  sim.commandMode(kPicoOutputLfoLo);
  sim.commandWave(kPicoLfoSine);
  sim.commandDepth(100);
  sim.commandRateStep(kPicoLfoLoRateSteps);
  sim.setPedalRaw(0);
  assert(sim.liveLfoRateHz() > 0.049f && sim.liveLfoRateHz() < 0.060f);
  sim.setPedalRaw(2048);
  assert(sim.liveLfoRateHz() > 0.90f && sim.liveLfoRateHz() < 1.12f);
  sim.setPedalRaw(4095);
  assert(sim.liveLfoRateHz() > 18.0f && sim.liveLfoRateHz() < 20.1f);
  sim.setPedalRaw(2048);
  int8_t previousSemitones = sim.currentSettings().semitones;
  sim.turnEncoder(-2);
  assert(sim.currentSettings().semitones == previousSemitones);
  assert(sim.currentSettings().lfoDepthPercent == 90);
  sim.commandDepth(100);
  sim.shortPressEncoder();
  sim.advanceMs(250);
  sim.printStatus("LO LFO one quarter cycle");
  expectNearMv("LO sine quarter cycle", sim.snapshot(), 3300.0f, 120.0f);
  sim.commandDepth(75);
  sim.shortPressEncoder();
  sim.advanceMs(250);
  sim.printStatus("LO LFO one quarter cycle depth 75");
  expectNearMv("LO sine quarter cycle depth 75", sim.snapshot(), 2887.5f, 120.0f);
  sim.commandDirection(kPicoBendDown);
  sim.advanceMs(250);
  sim.printStatus("LO LFO inverted");
  expectNearMv("LO sine inverted half cycle", sim.snapshot(), 1650.0f, 80.0f);
  sim.commandDirection(kPicoBendUp);
  sim.commandMode(kPicoOutputLfoFm);
  sim.commandWave(kPicoLfoSquare);
  sim.commandRateStep(kPicoLfoFmRateSteps);
  sim.setPedalRaw(4095);
  assert(sim.liveLfoRateHz() > 150.0f && sim.liveLfoRateHz() < 160.5f);
  sim.printStatus("FM square LFO");
  sim.commandMode(kPicoOutputPedal);

  sim.doubleClickEncoder();
  assert(sim.currentSettings().outputFocus == OutputFocusLfo1);
  uint8_t previousLfo1Step = sim.currentSettings().lfoOutLoRate[0];
  sim.turnEncoder(4);
  assert(sim.currentSettings().lfoOutLoRate[0] > previousLfo1Step);
  sim.commandWave(kPicoLfoSawUp);
  sim.commandDepth(75);
  sim.shortPressEncoder();
  sim.advanceMs(500);
  Snapshot lfo1Snapshot = sim.snapshot();
  assert(lfo1Snapshot.lfoOutDacCode[0] > 0);
  assert(sim.currentSettings().lfoOutWave[0] == kPicoLfoSawUp);
  assert(sim.currentSettings().lfoOutDepthPercent[0] == 75);
  sim.doubleClickEncoder();
  assert(sim.currentSettings().outputFocus == OutputFocusLfo2);
  sim.commandMode(kPicoOutputLfoFm);
  sim.commandRateStep(kPicoDefaultLfoFmRate);
  assert(sim.lfoOutLiveRateHz(1) >= kPicoLfoFmMinHz);
  sim.doubleClickEncoder();
  assert(sim.currentSettings().outputFocus == OutputFocusExp);

  sim.turnEncoder(24);
  assert(sim.currentSettings().semitones == kPicoMaxSemitones);
  sim.turnEncoder(-7);
  assert(sim.currentSettings().semitones == 2);
  sim.shortPressEncoder();
  assert(sim.currentSettings().semitones == 0);
  sim.advanceMs(3001);
  assert(!sim.isDirty());
  sim.printStatus("encoder center and autosave");

  sim.holdEncoder(2000);
  sim.turnEncoder(4);
  sim.shortPressEncoder();
  sim.setPedalRaw(137, 200);
  sim.shortPressEncoder();
  assert(sim.currentSettings().heelRaw == 137);
  sim.setPedalRaw(3788, 200);
  sim.shortPressEncoder();
  assert(sim.currentSettings().toeRaw == 3788);
  sim.commandInterval(9);
  sim.setPedalRaw(137);
  expectNearMv("calibrated heel", sim.snapshot(), 0.0f, 2.0f);
  sim.setPedalRaw(3788);
  expectNearMv("calibrated toe", sim.snapshot(), 3214.0f, 2.0f);
  sim.printStatus("calibrated custom pedal endpoints");

  sim.advanceMs(kIdleSleepMs + 1);
  assert(!sim.isSleeping());
  sim.printStatus("after one hour idle with bench idle sleep disabled");

  sim.setPedalRaw(2200, 100);
  assert(!sim.isSleeping());
  sim.printStatus("pedal movement stays awake");

  // LFO2 link with 180-degree phase offset: sawup pair a half cycle apart.
  sim.commandFocus(OutputFocusLfo1);
  sim.commandMode(kPicoOutputLfoLo);
  sim.commandWave(kPicoLfoSawUp);
  sim.commandDepth(100);
  sim.commandRateStep(kPicoDefaultLfoLoRate);
  sim.commandFocus(OutputFocusLfo2);
  sim.commandMode(kPicoOutputLfoLo);
  sim.commandWave(kPicoLfoSawUp);
  sim.commandDepth(100);
  sim.commandLink(kPicoLfoLink1to1);
  sim.commandLinkPhase(kPicoLfoPhase180);
  sim.commandSyncAll();
  sim.advanceMs(250);
  Snapshot linked = sim.snapshot();
  assert(fabsf(linked.lfoOutMicrovolts[0] / 1000.0f - 825.0f) < 40.0f);
  assert(fabsf(linked.lfoOutMicrovolts[1] / 1000.0f - 2475.0f) < 40.0f);
  sim.printStatus("LFO2 linked 1:1 at 180 degrees");

  // Clock square on channel D follows LFO1's half cycle.
  sim.commandClock(kPicoClockLfo1);
  sim.advanceMs(1);
  assert(sim.snapshot().clockDacCode == kMcp4728MaxCode);
  sim.advanceMs(500);
  assert(sim.snapshot().clockDacCode == 0);
  sim.commandClock(kPicoClockOff);

  // Depth 0 pins the output at the midpoint; offset shifts the center.
  sim.commandLink(kPicoLfoLinkOff);
  sim.commandDepth(0);
  sim.advanceMs(100);
  assert(fabsf(sim.snapshot().lfoOutMicrovolts[1] / 1000.0f - 1650.0f) < 5.0f);
  sim.commandOffset(25);
  sim.advanceMs(100);
  assert(fabsf(sim.snapshot().lfoOutMicrovolts[1] / 1000.0f - 2475.0f) < 5.0f);
  sim.commandOffset(0);
  sim.commandDepth(100);

  // Pulse width 75% keeps the wave high for three quarters of the cycle.
  sim.commandWave(kPicoLfoPulse);
  sim.commandPulseWidth(75);
  sim.commandRateStep(kPicoDefaultLfoLoRate);
  sim.commandSyncAll();
  sim.advanceMs(600);
  assert(sim.snapshot().lfoOutMicrovolts[1] > 3200000L);
  sim.advanceMs(200);
  assert(sim.snapshot().lfoOutMicrovolts[1] < 100000L);

  // Random waves stay in range and hold a new value each cycle.
  sim.commandWave(kPicoLfoSampleHold);
  sim.commandSyncAll();
  sim.advanceMs(100);
  int32_t shFirst = sim.snapshot().lfoOutMicrovolts[1];
  sim.advanceMs(1000);
  int32_t shSecond = sim.snapshot().lfoOutMicrovolts[1];
  assert(shFirst >= 0 && shFirst <= 3300000L);
  assert(shSecond >= 0 && shSecond <= 3300000L);
  assert(shFirst != shSecond);
  sim.commandWave(kPicoLfoDrift);
  sim.advanceMs(100);
  int32_t driftValue = sim.snapshot().lfoOutMicrovolts[1];
  assert(driftValue >= 0 && driftValue <= 3300000L);
  sim.printStatus("random waves, pulse width, depth 0 and offset");

  // Tap tempo: two presses 500ms apart set the focused LFO to ~2 Hz.
  sim.commandFocus(OutputFocusLfo1);
  sim.commandWave(kPicoLfoSine);
  sim.shortPressEncoder();
  sim.advanceMs(500);
  sim.shortPressEncoder();
  assert(fabsf(sim.lfoOutLiveRateHz(0) - 2.0f) < 0.15f);
  sim.printStatus("tap tempo set LFO1 near 2Hz");
  sim.commandFocus(OutputFocusExp);

  printf("\nSIM RESULT: PASS\n");
  printf("Verified: one-direction UP/DOWN 6th-limited rails, MCP4728-style EXP/LFO1/LFO2 DAC state, fixed CV override, curve modes, LO/FM pedal-rate and depth controls, dedicated LFO focus cycling, LFO2 link ratios with phase offsets, sample+hold and drift waves, pulse width, 0-100%% depth with center offset, channel-D clock square, sync all, tap tempo, OLED menu pickers, tune-mode toe trimming, DAC codes, OLED text output, encoder stepping, calibration, autosave, and bench idle-sleep disabled.\n");
}

}  // namespace

int main() {
  runScriptedSimulation();
  return 0;
}
