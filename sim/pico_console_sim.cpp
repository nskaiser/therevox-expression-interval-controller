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

enum CalibrationState {
  CalIdle,
  CalWaitHeel,
  CalWaitToe,
};

enum MenuItem {
  MenuMode,
  MenuWave,
  MenuDepth,
  MenuCurve,
  MenuCal,
  MenuDir,
  MenuDone,
  MenuCount,
};

enum MenuEditField {
  MenuEditNone,
  MenuEditMode,
  MenuEditWave,
  MenuEditDepth,
  MenuEditCurve,
};

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
  uint16_t rangeMv = 3300;
  uint16_t responseCents = kDefaultFullScaleResponseCents;
  uint16_t toeMapMv[kPicoSemitoneMapCount] = {};
};

struct Snapshot {
  int32_t outputMicrovolts = kDefaultCenterMicrovolts;
  uint16_t dacCode = 0;
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
    printf("SERIAL: MCP4725 %s at 0x62\n", dacPresent ? "PASS" : "FAIL");
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

  void commandCurve(uint8_t curveMode) {
    settings.curveMode = clampPicoCurveMode(curveMode);
    configureProcessor();
    markDirty();
    printf("SERIAL: OK curve %s\n", picoCurveName(settings.curveMode));
  }

  void commandMode(uint8_t outputMode) {
    settings.outputMode = clampPicoOutputMode(outputMode);
    lfoPhase = 0.0f;
    tuneMode = false;
    markDirty();
    printf("SERIAL: OK mode %s\n", picoOutputModeName(settings.outputMode));
  }

  void commandWave(uint8_t wave) {
    settings.lfoWave = clampPicoLfoWave(wave);
    lfoPhase = 0.0f;
    markDirty();
    printf("SERIAL: OK wave %s\n", picoLfoWaveName(settings.lfoWave));
  }

  void commandRateStep(int step) {
    if (settings.outputMode == kPicoOutputLfoFm) {
      settings.lfoFmRate = clampPicoLfoRateStep(step, settings.outputMode);
    } else {
      settings.lfoLoRate = clampPicoLfoRateStep(step, settings.outputMode);
    }
    markDirty();
    printf("SERIAL: OK max rate %.3fHz step=%u\n", configuredLfoMaxRateHz(), currentLfoRateStep());
  }

  void commandDepth(int percent) {
    settings.lfoDepthPercent = clampPicoLfoDepthPercent(percent);
    markDirty();
    printf("SERIAL: OK depth %u%%\n", settings.lfoDepthPercent);
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
          menuIndex = static_cast<MenuItem>(
              wrapMenuChoice(static_cast<int>(menuIndex) + direction, MenuCount));
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

    if (outputModeIsLfo()) {
      lfoPhase = 0.0f;
      printf("ENCODER: short press -> LFO sync\n");
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
    commandDirection(settings.bendDirection == kPicoBendDown ? kPicoBendUp : kPicoBendDown);
    save();
    printf("ENCODER: double click -> %s\n", directionLabel());
  }

  void holdEncoder(uint32_t holdMs) {
    if (sleeping) {
      wake("BUTTON");
      return;
    }

    if (holdMs >= 2000 && calState == CalIdle) {
      menuActive = !menuActive;
      menuIndex = MenuMode;
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

  Settings currentSettings() const {
    return settings;
  }

  void printStatus(const char* title) const {
    char label[8];
    signedIntervalLabel(label, sizeof(label));

    printf("\n== %s ==\n", title);
    printf("SERIAL: raw=%u norm=%.4f filt=%.4f interval=%s semis=%d mode=%s dir=%s noBendMv=%.3f outMv=%.3f dacCode=%u tune=%s stepMv=%u responseCents=%u curve=%s wave=%s rateHz=%.3f maxRateHz=%.3f depth=%u%% cv=%s dac=%s oled=%s cal=%s menu=%s sleep=%s heel=%u toe=%u invert=%s dirty=%s\n",
           pedalState.raw,
           pedalState.normalized,
           pedalState.filtered,
           label,
           settings.semitones,
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
  uint16_t dacCode = 0;
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
  MenuItem menuIndex = MenuMode;
  MenuEditField menuEditField = MenuEditNone;
  uint8_t menuEditValue = 0;
  float lfoPhase = 0.0f;
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
      lfoPhase -= floorf(lfoPhase);
      float value = computePicoLfoWaveValue(lfoPhase, settings.lfoWave);
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
    dacCode = microvoltsToMcp4725Code(outputMicrovolts);

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

  bool outputModeIsLfo() const {
    return settings.outputMode == kPicoOutputLfoLo || settings.outputMode == kPicoOutputLfoFm;
  }

  uint8_t currentLfoRateStep() const {
    return settings.outputMode == kPicoOutputLfoFm ? settings.lfoFmRate : settings.lfoLoRate;
  }

  float currentLfoRateHz() const {
    return computePicoLfoRateHzForPedal(settings.outputMode, pedalState.filtered, currentLfoRateStep());
  }

  float configuredLfoMaxRateHz() const {
    return computePicoLfoRateHz(settings.outputMode, currentLfoRateStep());
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
      case MenuMode:
        return "MODE";
      case MenuWave:
        return "WAVE";
      case MenuDepth:
        return "DEPTH";
      case MenuCurve:
        return "CURVE";
      case MenuCal:
        return "CAL";
      case MenuDir:
        return outputModeIsLfo() ? "POL" : "DIR";
      case MenuDone:
        return "DONE";
      default:
        return "MENU";
    }
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
      case MenuEditMode:
        return "SET MODE";
      case MenuEditWave:
        return "SET WAVE";
      case MenuEditDepth:
        return "SET DEP";
      case MenuEditCurve:
        return "SET CURVE";
      default:
        return "SET";
    }
  }

  uint8_t menuEditValueCount(MenuEditField field) const {
    switch (field) {
      case MenuEditMode:
        return kPicoOutputModeCount;
      case MenuEditWave:
        return kPicoLfoWaveCount;
      case MenuEditDepth:
        return kPicoLfoDepthStepCount;
      case MenuEditCurve:
        return kPicoCurveCount;
      default:
        return 1;
    }
  }

  uint8_t currentMenuEditValue(MenuEditField field) const {
    switch (field) {
      case MenuEditMode:
        return settings.outputMode;
      case MenuEditWave:
        return settings.lfoWave;
      case MenuEditDepth:
        return picoLfoDepthIndexFromPercent(settings.lfoDepthPercent);
      case MenuEditCurve:
        return settings.curveMode;
      default:
        return 0;
    }
  }

  const char* menuEditValueLabel(MenuEditField field, uint8_t value) const {
    switch (field) {
      case MenuEditMode:
        return picoOutputModeDisplayLabel(value);
      case MenuEditWave:
        return picoLfoWaveDisplayLabel(value);
      case MenuEditDepth: {
        static char depth[8];
        snprintf(depth, sizeof(depth), "%u%%", picoLfoDepthPercentFromIndex(value));
        return depth;
      }
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
      case MenuEditMode:
        commandMode(value);
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
      case MenuMode:
        beginMenuEdit(MenuEditMode);
        break;
      case MenuWave:
        beginMenuEdit(MenuEditWave);
        break;
      case MenuDepth:
        beginMenuEdit(MenuEditDepth);
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
        commandDirection(settings.bendDirection == kPicoBendDown ? kPicoBendUp : kPicoBendDown);
        save();
        printf("ENCODER: menu dir -> %s\n", directionLabel());
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
        snprintf(depthLabel, sizeof(depthLabel), "%u%%", settings.lfoDepthPercent);
        printf("  MENU     %s %s\n",
               menuItemTitle(menuIndex),
               menuIndex == MenuMode   ? picoOutputModeDisplayLabel(settings.outputMode)
               : menuIndex == MenuWave ? picoLfoWaveDisplayLabel(settings.lfoWave)
               : menuIndex == MenuDepth ? depthLabel
               : menuIndex == MenuCurve ? picoCurveDisplayLabel(settings.curveMode)
               : menuIndex == MenuDir   ? directionLabel()
                                        : "");
        printf("          TURN ITEM PRESS OK\n");
      }
      return;
    }

    char label[8];
    if (cvOverride) {
      snprintf(label, sizeof(label), "CV");
    } else if (outputModeIsLfo()) {
      snprintf(label, sizeof(label), "%s", picoOutputModeDisplayLabel(settings.outputMode));
    } else {
      signedIntervalLabel(label, sizeof(label));
    }

    printf("OLED:\n");
    if (outputModeIsLfo() && !cvOverride) {
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
    if (cvOverride) {
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
    if (outputModeIsLfo()) {
      printf("          %s %s\n",
             picoLfoWaveDisplayLabel(settings.lfoWave),
             settings.bendDirection == kPicoBendDown ? "DN" : "UP");
    } else {
      printf("          CRV %s\n", picoCurveDisplayLabel(settings.curveMode));
    }
    printf("  %s\n",
           dacPresent
               ? (cvOverride ? "CV HOLD" : (tuneMode ? "TUNE" : directionLabel()))
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

  sim.doubleClickEncoder();
  assert(sim.currentSettings().bendDirection == kPicoBendUp);
  assert(sim.currentSettings().semitones == kPicoMaxSemitones);
  sim.setPedalRaw(0);
  expectNearMv("double click direction up heel", sim.snapshot(), 0.0f, 2.0f);

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
  sim.turnEncoder(3);
  sim.shortPressEncoder();
  assert(sim.currentSettings().bendDirection == kPicoBendDown);
  sim.turnEncoder(1);
  sim.shortPressEncoder();
  assert(!sim.isMenuActive());
  sim.commandDirection(kPicoBendUp);
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
  sim.doubleClickEncoder();
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

  printf("\nSIM RESULT: PASS\n");
  printf("Verified: one-direction UP/DOWN 6th-limited rails, fixed CV override, curve modes, LO/FM pedal-rate and depth controls, LFO waveform controls, OLED menu pickers, tune-mode toe trimming, DAC codes, OLED text output, encoder stepping, double-click direction behavior, calibration, autosave, and bench idle-sleep disabled.\n");
}

}  // namespace

int main() {
  runScriptedSimulation();
  return 0;
}
