/*
 * Dose Coach - Handheld Exercise Tracker (final build)
 * Arduino UNO R4 Minima + Grove Base Shield (switch set to 5V)
 *
 * WIRING
 *   Grove-LCD RGB Backlight V4.0 -> any socket labelled I2C on the shield
 *   Grove Buzzer                 -> socket D8
 *   Grove Touch Sensor (TAP pad) -> socket D6   (main control, touched = HIGH)
 *   Button A                     -> pin D2, other side to GND (backup for the TAP pad)
 *   Button B                     -> pin D3, other side to GND
 *   Button C                     -> pin D4, other side to GND
 *   Status LED                   -> pin D7, through a 220 ohm resistor, to GND
 *   Red LED (warning light)      -> pin D5, through a 220 ohm resistor, to GND
 *                                   (on the breadboard; the Grove D5 socket stays empty)
 *   Connect to diagonally opposite legs on each button.
 *   Wrist sensor (MMA7361):
 *     sensor 5V  -> Arduino 5V        sensor SL -> sensor 3V3 (jumper on the sensor)
 *     sensor X   -> A0                sensor Y  -> A1
 *     sensor Z   -> A2                sensor GND -> GND next to Vin
 *     GS, ST, 0G -> nothing
 *
 * CONTROLS
 *   TAP pad or Button A : start / pause / resume a timer, or count one rep,
 *                         skip rest, or go to the next exercise when DONE.
 *                         In a sensor exercise (S): the first tap starts the set
 *                         and saves "arm down"; after that, extra taps add a rep
 *                         by hand (a backup if the sensor misses one).
 *   Button B : short press = next exercise, long press = previous exercise
 *   Button C : short press = restart exercise, long press = sound on/off
 *
 * LIGHTS
 *   Status LED (D7) : flashes with every beep, stays on when the exercise is done
 *   Red LED (D5)    : flashes for a warning (bad rep, rest cut, sensor problem),
 *                     stays on during an extra round of an exercise already finished
 *
 * SENSOR REPS (mode S, bicep curls)
 *   The angle is how far the forearm has swung up from "arm down" (0 degrees).
 *   A movement starts above 40 deg and ends below 25 deg. Then:
 *     reached 100 deg, took 1.5 s or more   -> counted              (short beep)
 *     reached 100 deg, took under 1.5 s     -> counted, "Too fast"  (red flash, warning beep)
 *     under 100 deg, arm swung fast         -> NOT counted, "Too fast"
 *     under 100 deg, normal speed           -> NOT counted, "Go higher"
 *   If the sensor reads impossible values for 1 s, the screen says "Sensor?",
 *   the red LED flashes, and taps count the reps instead. "Sensor OK" when it is back.
 *
 * WORKOUT
 *   The exercise list is in workout.h. Lines with a mistake are skipped, and
 *   Serial Monitor says which line and why.
 *
 * LOG
 *   Every set is printed to Serial Monitor as one CSV row:
 *     time_s,exercise,activity,set,target,done,took_s,note,
 *     counted_by,partial,too_fast,avg_peak_deg,avg_rep_s
 *   note is: ok, extra, stopped, left early, or rest cut.
 *   counted_by: sensor, tap, mixed (both), or - (timer and rest rows)
 *   partial: movements NOT counted because they stayed under 100 degrees
 *   too_fast: counted reps that were too fast
 *   avg_peak_deg, avg_rep_s: averages over every movement the sensor judged in
 *   that set (empty when the sensor judged none)
 *   Lines starting with # are status messages, not part of the log.
 *
 * No LCD library is needed. This board's SCL line has no working pull-up, so
 * the Wire library times out. The sketch talks to the LCD itself: it drives SCL
 * both high and low, and it uses the pin's pull-up on SDA.
 *
 * Open Serial Monitor at 115200 baud.
 */

#include <math.h>
#include "workout.h"

// ---------- pins ----------
const uint8_t PIN_BTN_A     = 2;
const uint8_t PIN_BTN_B     = 3;
const uint8_t PIN_BTN_C     = 4;
const uint8_t PIN_RED_LED   = 5;
const uint8_t PIN_TOUCH     = 6;
const uint8_t PIN_GREEN_LED = 7;           // the status LED (yellow or green)
const uint8_t PIN_BUZZER    = 8;
const uint8_t PIN_LCD_SDA   = A4;
const uint8_t PIN_LCD_SCL   = A5;
const uint8_t PIN_AXIS[3]   = {A0, A1, A2};   // sensor X, Y, Z
const char    AXIS_NAME[3]  = {'X', 'Y', 'Z'};

// ---------- switches ----------
const bool USE_TOUCH     = true;    // false = ignore the TAP pad. Set this if the pad is
                                    // unplugged: its pin then "floats" and reads random taps.
const bool PRINT_BUTTONS = false;   // true = print each button press (wiring test)
const bool PRINT_REPS    = true;    // true = print a # line for every sensor movement

const uint8_t  LCD_TEXT_ADDRESS  = 0x3E;   // text chip on the Grove LCD
const uint8_t  LCD_LIGHT_ADDRESS = 0x62;   // backlight chip on the V4.0 board
const uint16_t LCD_RETRY_MS    = 3000;
const uint16_t DEBOUNCE_MS     = 30;
const uint16_t LONG_PRESS_MS   = 800;
const uint16_t HELD_REPORT_MS  = 5000;     // a press held this long is reported once
const uint16_t TAP_GUARD_MS    = 1000;     // after a set ends, ignore taps this long, so one
                                           // tap too many can't cut the rest or skip WELL DONE
const uint16_t MESSAGE_MS      = 1200;
const uint16_t WARNING_MS      = 3000;
const uint16_t GOOD_MS         = 800;      // "Good!" after a good rep
const uint16_t PROMPT_MS       = 1800;     // READY screen: how long each hint shows
const uint16_t CELEBRATE_MS    = 3500;     // "WELL DONE!" animation after an exercise
const uint16_t SCREEN_EVERY_MS = 50;       // redraw the screen at most this often

const uint8_t MAX_EXERCISES = 30;
const uint8_t NAME_LENGTH   = 16;          // fits one LCD line

// ---------- rep rules (tuned with CurlTest) ----------
const float    REP_START_DEG  = 40.0;   // a movement starts when the angle rises above this
const float    REP_TOP_DEG    = 100.0;  // it must reach this to count
const float    REP_END_DEG    = 25.0;   // it ends when the angle drops below this
const uint16_t TOO_FAST_MS    = 1500;   // a counted rep shorter than this is "Too fast"
const uint16_t BUMP_MS        = 300;    // shorter than this = a bump or knock, ignored
const float    FAST_DEG_PER_S = 450;    // arm speed above this = a fast swing
                                        // (Nov 18 test: slow full curls 275-330 deg/s, fast 785, halves 142-362)
// The gap between 40 and 25 degrees stops one wobbly rep from counting twice.
// A movement's time runs from leaving 25 degrees on the way up to dropping
// back below 25 degrees, so it is close to the whole rep.

// ---------- sensor calibration (measured Nov 6) ----------
const float ZERO_VOLTS[3]  = {1.67, 1.87, 1.47};  // output voltage at 0 g, per axis
const float VOLTS_PER_G[3] = {0.83, 0.80, 0.78};  // change in voltage per 1 g, per axis
const float ADC_VOLTS = 5.0;     // the ADC (analog-to-digital converter) measures against 5 V
const float ADC_STEPS = 4095.0;  // 12-bit reading: 0 ... 4095

// ---------- sensor reading and smoothing (noise handling) ----------
const uint8_t  SAMPLES_PER_AXIS = 16;   // each reading = average of 16 samples per axis
const uint16_t READ_EVERY_MS    = 20;   // one averaged reading every 20 ms (50 per second)
const float    SMOOTHING        = 0.3;  // 0-1: how much each new reading counts

// ---------- sensor health check ----------
const float    MIN_GOOD_VOLTS = 0.15;   // the chip runs on 3.3 V: outside 0.15-3.20 V
const float    MAX_GOOD_VOLTS = 3.20;   //   means a loose wire or Sleep not at 3.3 V
const float    MIN_GOOD_G     = 0.4;    // a real arm doing curls stays inside 0.4-2.5 g
const float    MAX_GOOD_G     = 2.5;
const float    SAME_VOLTS     = 0.03;   // all 3 axes this close = pins floating (unplugged)
const uint16_t BAD_FOR_MS     = 1000;   // bad this long  -> "Sensor?"
const uint16_t GOOD_FOR_MS    = 500;    // good this long -> "Sensor OK"

// ---------- saving "arm down" ----------
const uint16_t ZERO_SETTLE_MS    = 300;   // ignore this long after the tap (tapping shakes the body)
const uint16_t ZERO_MS           = 500;   // then the arm must be still for this long
const uint16_t ZERO_GIVE_UP_MS   = 4000;  // keep waiting for a still moment this long, then "Tap again"
const float    ZERO_MAX_SPREAD_G = 0.20;  // an axis moving more than this = arm not still
const float    ZERO_MIN_G        = 0.8;   // a still arm reads close to 1 g in total
const float    ZERO_MAX_G        = 1.2;

// ---------- screen: custom characters and the angle bar ----------
// The LCD has 8 slots for home-made characters. Slot 0 is not used, because a
// character code of 0 would end a text string.
const char CH_BAR1 = 1, CH_BAR2 = 2, CH_BAR3 = 3, CH_BAR4 = 4;  // bar cell with 1-4 columns lit
const char CH_TARGET = 5;            // dotted line: the 100 degree target
const char CH_DOWN   = 6;            // arrow pointing down
const char CH_STAR   = 7;            // star for the celebration
const char CH_FULL   = (char)0xFF;   // solid block, built into the LCD
const char CH_DEGREE = (char)0xDF;   // degree sign, built into the LCD
const uint8_t GLYPHS[7][8] = {       // 8 rows of 5 dots each; 1 = lit
  {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10},   // 1 column
  {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},   // 2 columns
  {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C},   // 3 columns
  {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E},   // 4 columns
  {0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00},   // dotted target line
  {0x04, 0x04, 0x04, 0x04, 0x15, 0x0E, 0x04, 0x00},   // down arrow
  {0x04, 0x04, 0x1F, 0x0E, 0x0E, 0x1B, 0x11, 0x00},   // star
};
const float   BAR_FULL_DEG = 125.0;  // the 10-cell bar is full at 125 degrees (12.5 per cell)
const uint8_t TARGET_CELL  = 8;      // 8 cells x 12.5 = 100 degrees: the target mark
const uint8_t PARTY_COLOURS[6][3] = {
  {255, 0, 0}, {255, 140, 0}, {255, 255, 0}, {0, 220, 60}, {0, 90, 255}, {170, 0, 255}
};

// ---------- types ----------
enum Mode : uint8_t { TIMER, REPS, SENSOR };
enum State : uint8_t { READY, RUNNING, PAUSED, RESTING, DONE };
enum Press : uint8_t { NO_PRESS, SHORT_PRESS, LONG_PRESS };
enum RepResult : uint8_t { NO_REP, REP_GOOD, REP_TOO_FAST, REP_FAST_LOW, REP_TOO_LOW, REP_BUMP };

struct Exercise {
  char     name[NAME_LENGTH + 1];
  Mode     mode;     // TIMER, REPS or SENSOR
  uint16_t target;   // seconds for TIMER, reps for REPS and SENSOR
  uint8_t  sets;
  uint16_t rest;     // seconds between sets
};

struct Button {
  uint8_t  pin;
  const char *label;
  bool     activeHigh;   // true: HIGH = pressed (TAP pad). false: LOW = pressed (buttons)
  bool     down;
  bool     longSent;
  bool     heldReported;
  uint32_t changedAt;
  uint32_t pressedAt;
};

// Quality numbers for the set in progress (sensor exercises).
struct SetStats {
  uint16_t sensorReps;   // reps counted by the sensor
  uint16_t tapReps;      // reps counted by a tap
  uint16_t partial;      // movements not counted (under 100 degrees)
  uint16_t tooFast;      // counted reps that were too fast
  uint16_t moves;        // every movement the sensor judged
  float    peakSum;      // for the average peak angle
  uint32_t msSum;        // for the average rep time
};

// ---------- sounds: on, off, on, off ... ending in 0 (milliseconds) ----------
const uint16_t SOUND_CLICK[]  = {25, 0};
const uint16_t SOUND_START[]  = {250, 0};
const uint16_t SOUND_TICK[]   = {70, 0};
const uint16_t SOUND_HALF[]   = {90, 80, 90, 0};
const uint16_t SOUND_SET[]    = {250, 100, 250, 0};
const uint16_t SOUND_FINISH[] = {200, 80, 200, 80, 200, 80, 600, 0};
const uint16_t SOUND_WARN[]   = {400, 150, 400, 0};

// ---------- variables ----------
bool lcdFound = false;

Exercise workout[MAX_EXERCISES];
uint8_t  exerciseCount = 0;
uint8_t  workoutErrors = 0;
bool     finished[MAX_EXERCISES];      // exercise completed this session

Button btnA   = {PIN_BTN_A, "A (D2)",       false, false, false, false, 0, 0};
Button btnB   = {PIN_BTN_B, "B (D3)",       false, false, false, false, 0, 0};
Button btnC   = {PIN_BTN_C, "C (D4)",       false, false, false, false, 0, 0};
Button btnPad = {PIN_TOUCH, "TAP pad (D6)", true,  false, false, false, 0, 0};
bool   padProbeHigh = false;           // startup plug-in check of the TAP pad

uint8_t  exerciseIndex = 0;
State    state = READY;
uint8_t  setNumber = 1;
uint16_t repCount = 0;
uint32_t endTime = 0;
uint32_t pausedTimeLeft = 0;
uint16_t lastSecond = 0;
uint32_t setStartedAt = 0;
bool     extraRound = false;           // repeating an exercise already done
uint32_t warningUntil = 0;
SetStats stats;
uint16_t exReps = 0;                   // reps counted in this exercise (all sets)
uint16_t exGood = 0;                   // good sensor reps in this exercise
uint16_t exTries = 0;                  // every rep tried: counted ones plus ones not counted
uint16_t wkReps = 0;                   // whole workout: reps tried in finished sensor exercises
uint16_t wkGood = 0;                   // of those, good reps (for the "Workout done!" screen)
uint32_t celebrateUntil = 0;           // end of the "WELL DONE!" animation
uint32_t tapsIgnoredUntil = 0;         // TAP pad and button A ignored until then
char     shownLine[2][17];             // what is on the LCD now (for partial redraws)
int      shownColour = -1;             // backlight colour now on the LCD

bool     muted = false;
const char *message = "";
uint32_t messageUntil = 0;

const uint16_t *sound = nullptr;
uint8_t  soundStep = 0;
uint32_t nextSoundStep = 0;
bool     soundOn = false;

// sensor readings
float volts[3]   = {0, 0, 0};          // latest averaged voltage, per axis
float rawG[3]    = {0, 0, 0};          // latest reading in g (not smoothed)
float smoothG[3] = {0, 0, 0};          // smoothed reading in g
bool  haveSmooth = false;
float totalG     = 0;                  // length of the smoothed gravity arrow, in g
float angleDeg   = 0;                  // angle from "arm down", in degrees
uint32_t lastReadAt = 0;
uint32_t firstReadAt = 0;             // when the sensor was first read (0 = not yet)

// sensor health. Assumed OK at power-up; 1 s of bad readings proves otherwise.
bool        sensorOk    = true;
bool        badRunning  = false;
bool        goodRunning = false;
uint32_t    badSince    = 0;
uint32_t    goodSince   = 0;
const char *lastProblem = "";

// "arm down" reference, saved at the start of every sensor set
bool     haveRef = false;
float    ref[3]  = {0, 0, 0};          // unit-length arrow: gravity direction with arm down
bool     zeroing = false;
uint32_t zeroStartedAt = 0;             // when the tap happened
uint32_t zeroWindowAt = 0;              // when the current still half-second began
uint16_t zeroCount = 0;
float    zeroSum[3], zeroMin[3], zeroMax[3];

// rep checker
bool     armUp = false;                // inside a movement (angle went above 40)
uint32_t moveStartedAt = 0;
uint32_t lastBelowEndAt = 0;
float    movePeak = 0;
float    moveTopSpeed = 0;
float    lastPeak = 0;
uint32_t lastTookMs = 0;
float    recentAngles[4] = {0, 0, 0, 0};   // last 4 angles and their times, for the speed
uint32_t recentAt[4] = {0, 0, 0, 0};
uint8_t  recentIndex = 0;
float    armSpeed = 0;                     // degrees per second

const Exercise &current() { return workout[exerciseIndex]; }

// Written further down; declared here so earlier functions can call them.
void startZeroing();
void finishSet();
void forgetScreen();

// True while the current exercise is a sensor exercise and a set is running.
bool inSensorSet() { return exerciseCount > 0 && current().mode == SENSOR && state == RUNNING; }

// True when every exercise in the plan has been finished this round.
bool workoutComplete() {
  if (exerciseCount == 0) return false;
  for (uint8_t i = 0; i < exerciseCount; i++) if (!finished[i]) return false;
  return true;
}

// True when the sensor can't be used for this set, so taps count the reps.
bool countingByTaps() { return inSensorSet() && (!sensorOk || !haveRef); }

// ---------- reading workout.h ----------
char *trimSpaces(char *text) {
  while (*text == ' ' || *text == '\t') text++;
  char *end = text + strlen(text);
  while (end > text && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r')) *--end = 0;
  return text;
}

// Cuts the next value off the line (up to the next comma).
char *nextField(char *&cursor) {
  if (cursor == nullptr) return nullptr;
  char *start = cursor;
  char *comma = strchr(cursor, ',');
  if (comma != nullptr) {
    *comma = 0;
    cursor = comma + 1;
  } else {
    cursor = nullptr;
  }
  return trimSpaces(start);
}

// True if text is a whole number from low to high.
bool readNumber(const char *text, uint16_t low, uint16_t high, uint16_t &value) {
  if (*text == 0) return false;
  uint32_t number = 0;
  for (; *text != 0; text++) {
    if (*text < '0' || *text > '9') return false;
    number = number * 10 + (*text - '0');
    if (number > high) return false;
  }
  if (number < low) return false;
  value = (uint16_t)number;
  return true;
}

// Reads one line. Returns nullptr if it is fine, otherwise why it was skipped.
const char *parseLine(char *line, Exercise &out, bool &isExercise) {
  isExercise = false;
  char *cursor = trimSpaces(line);
  if (*cursor == 0 || *cursor == '#') return nullptr;   // blank or comment

  char *name   = nextField(cursor);
  char *mode   = nextField(cursor);
  char *target = nextField(cursor);
  char *sets   = nextField(cursor);
  char *rest   = nextField(cursor);
  if (cursor != nullptr) return "more than 5 values";
  if (mode == nullptr || target == nullptr) return "needs at least name,mode,target";

  size_t length = strlen(name);
  if (length == 0) return "name is empty";
  if (length > NAME_LENGTH) return "name is longer than 16 characters";
  strcpy(out.name, name);

  if (strlen(mode) != 1) return "mode must be T, C or S";
  char m = mode[0];
  if (m == 'T' || m == 't') out.mode = TIMER;
  else if (m == 'C' || m == 'c') out.mode = REPS;
  else if (m == 'S' || m == 's') out.mode = SENSOR;
  else return "mode must be T, C or S";

  if (out.mode == TIMER) {
    if (!readNumber(target, 1, 5999, out.target)) return "timer target must be 1-5999 seconds";
  } else {
    if (!readNumber(target, 1, 999, out.target)) return "rep target must be 1-999";
  }

  uint16_t number = 1;
  if (sets != nullptr && *sets != 0 && !readNumber(sets, 1, 99, number)) return "sets must be 1-99";
  out.sets = (uint8_t)number;

  out.rest = 0;
  if (rest != nullptr && *rest != 0 && !readNumber(rest, 0, 999, out.rest)) return "rest must be 0-999 seconds";

  isExercise = true;
  return nullptr;
}

// Goes through BUILTIN_CSV line by line. keep = fill the workout list,
// report = print skipped lines. Returns how many lines were skipped.
uint8_t readWorkout(bool keep, bool report) {
  char line[80];
  uint8_t length = 0;
  bool tooLong = false;
  uint8_t found = 0;
  uint8_t errors = 0;

  for (size_t i = 0; ; i++) {
    char c = BUILTIN_CSV[i];             // PROGMEM data can be read directly on the R4
    if (c != '\n' && c != 0) {
      if (length < sizeof line - 1) line[length++] = c;
      else tooLong = true;
      continue;
    }
    line[length] = 0;

    char original[sizeof line];
    strcpy(original, line);
    Exercise exercise;
    bool isExercise = false;
    const char *problem;
    if (tooLong) problem = (*trimSpaces(line) == '#') ? nullptr : "line is too long";  // a long comment is fine
    else         problem = parseLine(line, exercise, isExercise);

    if (problem == nullptr && isExercise) {
      if (found >= MAX_EXERCISES) {
        problem = "more than 30 exercises";
      } else {
        if (keep) workout[found] = exercise;
        found++;
      }
    }
    if (problem != nullptr) {
      errors++;
      if (report) {
        Serial.print(F("# Skipped line in workout.h ("));
        Serial.print(problem);
        Serial.print(F("): "));
        Serial.println(trimSpaces(original));
      }
    }

    length = 0;
    tooLong = false;
    if (c == 0) break;
  }

  if (keep) exerciseCount = found;
  return errors;
}

// ---------- sound and lights (never blocks) ----------
void setOutputs(bool on) {
  soundOn = on;                                   // status LED follows the sound
  digitalWrite(PIN_BUZZER, (on && !muted) ? HIGH : LOW);
}

void playSound(const uint16_t *pattern) {
  setOutputs(false);
  sound = pattern;
  soundStep = 0;
  nextSoundStep = millis();
}

void updateSound() {
  if (sound == nullptr || (int32_t)(millis() - nextSoundStep) < 0) return;
  uint16_t duration = sound[soundStep];
  if (duration == 0) {
    setOutputs(false);
    sound = nullptr;
    return;
  }
  setOutputs(soundStep % 2 == 0);
  nextSoundStep = millis() + duration;
  soundStep++;
}

void updateLights() {
  bool warning = (int32_t)(millis() - warningUntil) < 0;
  bool green = soundOn || (exerciseCount > 0 && state == DONE);
  bool red = extraRound || (warning && (millis() / 250) % 2 == 0);
  digitalWrite(PIN_GREEN_LED, green ? HIGH : LOW);
  digitalWrite(PIN_RED_LED, red ? HIGH : LOW);
}

void showMessage(const char *text, uint16_t ms) {
  message = text;
  messageUntil = millis() + ms;
}

void warn(const char *text) {
  showMessage(text, WARNING_MS);
  warningUntil = millis() + WARNING_MS;
  playSound(SOUND_WARN);
}

// ---------- workout logic ----------
uint32_t timeLeftMs() {
  if (state == PAUSED) return pausedTimeLeft;
  int32_t left = (int32_t)(endTime - millis());
  return left > 0 ? (uint32_t)left : 0;
}

uint16_t secondsLeft() { return (uint16_t)((timeLeftMs() + 999) / 1000); }

void startCountdown(uint32_t ms) {
  endTime = millis() + ms;
  lastSecond = (uint16_t)((ms + 999) / 1000);
}

// ---------- CSV log ----------
void printLogHeader() {
  Serial.println(F("time_s,exercise,activity,set,target,done,took_s,note,"
                   "counted_by,partial,too_fast,avg_peak_deg,avg_rep_s"));
}

// The five quality columns at the end of a row.
void printQualityColumns(bool forThisSet) {
  if (!forThisSet || current().mode == TIMER) { Serial.println(F("-,0,0,,")); return; }
  if (current().mode == REPS)                 { Serial.println(F("tap,0,0,,")); return; }

  if (stats.sensorReps > 0 && stats.tapReps > 0) Serial.print(F("mixed"));
  else if (stats.sensorReps > 0)                 Serial.print(F("sensor"));
  else if (stats.tapReps > 0)                    Serial.print(F("tap"));
  else                                           Serial.print(F("-"));
  Serial.print(',');
  Serial.print(stats.partial);  Serial.print(',');
  Serial.print(stats.tooFast);  Serial.print(',');
  if (stats.moves > 0) {
    Serial.print(stats.peakSum / stats.moves, 0);
    Serial.print(',');
    Serial.println(stats.msSum / 1000.0 / stats.moves, 1);
  } else {
    Serial.println(',');                      // both averages empty
  }
}

// forThisSet = false for rest rows and for sets that never started.
void logRow(const char *activity, uint8_t set, uint16_t target, uint16_t done,
            uint32_t took, const char *note, bool forThisSet) {
  Serial.print(millis() / 1000);   Serial.print(',');
  Serial.print(current().name);    Serial.print(',');
  Serial.print(activity);          Serial.print(',');
  Serial.print(set);               Serial.print('/');
  Serial.print(current().sets);    Serial.print(',');
  Serial.print(target);            Serial.print(',');
  Serial.print(done);              Serial.print(',');
  Serial.print(took);              Serial.print(',');
  Serial.print(note);              Serial.print(',');
  printQualityColumns(forThisSet);
}

const char *activityName() {
  if (current().mode == TIMER) return "timer";
  if (current().mode == SENSOR) return "sensor reps";
  return "reps";
}

// Reps counted, or whole seconds the timer has run, in the current set.
uint16_t doneSoFar() {
  if (current().mode != TIMER) return repCount;
  uint32_t total = (uint32_t)current().target * 1000UL;
  uint32_t left = timeLeftMs();
  return (uint16_t)((total - (left < total ? left : total)) / 1000);
}

uint32_t setSeconds() { return (millis() - setStartedAt) / 1000; }

// Logs unfinished work when the participant leaves an exercise early.
void logIfLeftEarly() {
  if (exerciseCount == 0 || state == DONE) return;
  if (state == RUNNING || state == PAUSED) {
    logRow(activityName(), setNumber, current().target, doneSoFar(), setSeconds(), "stopped", true);
  } else if (state == RESTING) {
    logRow(activityName(), setNumber + 1, current().target, 0, 0, "left early", false);
  } else if (setNumber > 1) {
    logRow(activityName(), setNumber, current().target, 0, 0, "left early", false);
  }
}

void prepareSet() {
  repCount = 0;
  state = READY;
  memset(&stats, 0, sizeof stats);
  haveRef = false;                      // every sensor set saves its own "arm down"
  zeroing = false;
  armUp = false;
}

void goToExercise(int16_t index) {
  logIfLeftEarly();
  if (index < 0) index = exerciseCount - 1;
  if (index >= exerciseCount) index = 0;
  exerciseIndex = (uint8_t)index;
  setNumber = 1;
  extraRound = false;
  exReps = exGood = exTries = 0;
  celebrateUntil = 0;
  prepareSet();
  playSound(SOUND_CLICK);
}

void nextSet() {
  setNumber++;
  prepareSet();
}

void finishSet() {
  logRow(activityName(), setNumber, current().target, doneSoFar(), setSeconds(),
         extraRound ? "extra" : "ok", true);
  exReps += doneSoFar();
  exGood += stats.sensorReps - stats.tooFast;
  exTries += doneSoFar() + stats.partial;
  tapsIgnoredUntil = millis() + TAP_GUARD_MS;
  if (setNumber >= current().sets) {
    state = DONE;
    finished[exerciseIndex] = true;
    if (current().mode == SENSOR) {           // add this exercise to the workout totals
      wkReps += exTries;
      wkGood += exGood;
    }
    celebrateUntil = millis() + CELEBRATE_MS;   // start the "WELL DONE!" screen
    playSound(SOUND_FINISH);
    if (workoutComplete()) {
      Serial.print(F("# Workout complete: all "));
      Serial.print(exerciseCount);
      Serial.print(F(" exercises done"));
      if (wkReps > 0) {
        Serial.print(F(", good reps "));
        Serial.print(wkGood);
        Serial.print('/');
        Serial.print(wkReps);
        Serial.print(F(" ("));
        Serial.print(100UL * wkGood / wkReps);
        Serial.print(F(" %)"));
      }
      Serial.println('.');
    }
  } else if (current().rest > 0) {
    state = RESTING;
    startCountdown((uint32_t)current().rest * 1000UL);
    showMessage("Set done!", MESSAGE_MS);
    playSound(SOUND_SET);
  } else {
    nextSet();
    showMessage("Set done!", MESSAGE_MS);
    playSound(SOUND_SET);
  }
}

void updateCountdown() {
  bool exerciseTimer = (state == RUNNING && current().mode == TIMER);
  if (!exerciseTimer && state != RESTING) return;

  uint16_t second = secondsLeft();
  if (second != lastSecond) {
    lastSecond = second;
    if (second > 0 && second <= 3) {
      playSound(SOUND_TICK);
    } else if (exerciseTimer && current().target >= 20 && second == current().target / 2) {
      playSound(SOUND_HALF);
    }
  }

  if (timeLeftMs() == 0) {
    if (exerciseTimer) {
      finishSet();
    } else {
      nextSet();
      playSound(SOUND_START);
    }
  }
}

// ---------- buttons and the TAP pad ----------
Press readButton(Button &b) {
  bool level   = (digitalRead(b.pin) == HIGH);
  bool pressed = b.activeHigh ? level : !level;  // buttons use INPUT_PULLUP: LOW = pressed
  uint32_t now = millis();

  if (pressed != b.down && now - b.changedAt >= DEBOUNCE_MS) {
    b.changedAt = now;
    b.down = pressed;
    if (pressed) {
      b.pressedAt = now;
      b.longSent = false;
      b.heldReported = false;
      if (PRINT_BUTTONS) {
        Serial.print(F("# Button pressed: "));
        Serial.println(b.label);
      }
    } else if (!b.longSent) {
      return SHORT_PRESS;
    }
  } else if (b.down && !b.longSent && now - b.pressedAt >= LONG_PRESS_MS) {
    b.longSent = true;
    return LONG_PRESS;
  } else if (b.down && !b.heldReported && now - b.pressedAt >= HELD_REPORT_MS) {
    // Held for 5 s: a stuck button, or a hand resting on the pad. Reported once.
    b.heldReported = true;
    Serial.print(F("# "));
    Serial.print(b.label);
    Serial.println(F(" held for 5 s: stuck, or a hand resting on it?"));
    showMessage("Held 5 s?", WARNING_MS);
  }
  return NO_PRESS;
}

void startSensorSet() {
  state = RUNNING;
  setStartedAt = millis();
  if (finished[exerciseIndex] && !extraRound) {
    extraRound = true;                      // already did this one today
    showMessage("Extra!", MESSAGE_MS);
  }
}

void handleButtonA() {
  switch (state) {
    case READY:
      if (current().mode == SENSOR) {
        if (zeroing) break;                 // already saving "arm down"
        if (sensorOk) {
          startZeroing();                   // the set starts once "arm down" is saved
          playSound(SOUND_CLICK);
        } else {
          startSensorSet();                 // no sensor: taps count the reps
          warn("Sensor?");
          Serial.println(F("# Sensor not OK: this set counts taps."));
        }
        break;
      }
      state = RUNNING;
      setStartedAt = millis();
      if (finished[exerciseIndex] && !extraRound) {
        extraRound = true;                    // already did this one today
        showMessage("Extra!", MESSAGE_MS);
      }
      if (current().mode == TIMER) {
        startCountdown((uint32_t)current().target * 1000UL);
        playSound(SOUND_START);
      } else {
        repCount = 1;
        playSound(SOUND_CLICK);
        if (repCount >= current().target) finishSet();
      }
      break;
    case RUNNING:
      if (current().mode == TIMER) {
        pausedTimeLeft = timeLeftMs();
        state = PAUSED;
        playSound(SOUND_CLICK);
      } else {
        repCount++;                           // a rep counted by a tap
        if (current().mode == SENSOR) stats.tapReps++;
        playSound(SOUND_CLICK);
        if (repCount >= current().target) finishSet();
      }
      break;
    case PAUSED:
      state = RUNNING;
      startCountdown(pausedTimeLeft);
      playSound(SOUND_CLICK);
      break;
    case RESTING: {
      uint32_t restMs = (uint32_t)current().rest * 1000UL;
      uint32_t left = timeLeftMs();
      uint16_t rested = (uint16_t)((restMs - (left < restMs ? left : restMs)) / 1000);
      logRow("rest", setNumber, current().rest, rested, rested, "rest cut", false);
      nextSet();
      warn("Rest cut");
      break;
    }
    case DONE:
      if (exerciseIndex + 1 >= exerciseCount) {
        // Whole workout done: start a fresh round from the top, so the
        // exercises aren't all marked "Extra" (red LED on) the second time.
        for (uint8_t i = 0; i < exerciseCount; i++) finished[i] = false;
        wkReps = wkGood = 0;
        Serial.println(F("# Workout complete. Starting a new round from the first exercise."));
        goToExercise(0);
        showMessage("New round", MESSAGE_MS);
      } else {
        goToExercise(exerciseIndex + 1);
      }
      break;
  }
}

void handleButtonB(Press press) {
  goToExercise(press == LONG_PRESS ? exerciseIndex - 1 : exerciseIndex + 1);
}

void handleButtonC(Press press) {
  if (press == LONG_PRESS) {
    muted = !muted;
    showMessage(muted ? "Sound OFF" : "Sound ON", MESSAGE_MS);
  } else {
    logIfLeftEarly();
    setNumber = 1;
    extraRound = false;
    exReps = exGood = exTries = 0;
    celebrateUntil = 0;
    prepareSet();
    showMessage("Restarted", MESSAGE_MS);
  }
  playSound(SOUND_CLICK);
}

// Plug-in check for the TAP pad, at power-up. With the pin's pull-up on, an
// unplugged pin reads HIGH; a plugged-in pad that isn't touched drives it LOW.
// Only reported in Serial Monitor; USE_TOUCH is the real switch.
void probeTouchPad() {
  pinMode(PIN_TOUCH, INPUT_PULLUP);
  delay(5);
  padProbeHigh = (digitalRead(PIN_TOUCH) == HIGH);
  pinMode(PIN_TOUCH, INPUT);
}

// ---------- wrist sensor ----------
// Reads each axis 16 times and averages, which cuts random noise by about 4x.
void readSensorVolts() {
  for (uint8_t axis = 0; axis < 3; axis++) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < SAMPLES_PER_AXIS; i++) sum += analogRead(PIN_AXIS[axis]);
    volts[axis] = (sum / (float)SAMPLES_PER_AXIS) * ADC_VOLTS / ADC_STEPS;
    rawG[axis]  = (volts[axis] - ZERO_VOLTS[axis]) / VOLTS_PER_G[axis];
  }
}

float lengthOf(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// Returns nullptr if the reading looks possible, otherwise a short reason.
const char *readingProblem() {
  for (uint8_t axis = 0; axis < 3; axis++) {
    if (volts[axis] < MIN_GOOD_VOLTS) return "a voltage is near 0 V (loose wire, or Sleep not at 3.3 V)";
    if (volts[axis] > MAX_GOOD_VOLTS) return "a voltage is above 3.2 V (loose GND wire?)";
  }
  float lowest  = fminf(volts[0], fminf(volts[1], volts[2]));
  float highest = fmaxf(volts[0], fmaxf(volts[1], volts[2]));
  if (highest - lowest < SAME_VOLTS) return "all 3 axes read the same (sensor unplugged? pins floating)";
  if (totalG < MIN_GOOD_G) return "total g is too small";
  if (totalG > MAX_GOOD_G) return "total g is too big";
  return nullptr;
}

// Prints e.g. "X 1.66 Y 1.88 Z 2.25 V, total 1.00 g".
void printSensorValues() {
  for (uint8_t axis = 0; axis < 3; axis++) {
    Serial.print(AXIS_NAME[axis]);
    Serial.print(' ');
    Serial.print(volts[axis], 2);
    Serial.print(' ');
  }
  Serial.print(F("V, total "));
  Serial.print(totalG, 2);
  Serial.println(F(" g"));
}

// One bad reading is ignored. Bad for 1 s -> "Sensor?". Good for 0.5 s -> "Sensor OK".
// The two different times stop the message flickering.
void updateHealth(const char *problem, uint32_t now) {
  if (problem != nullptr) {
    lastProblem = problem;
    goodRunning = false;
    if (!badRunning) { badRunning = true; badSince = now; }
    if (sensorOk && now - badSince >= BAD_FOR_MS) {
      sensorOk = false;
      armUp = false;                    // drop any half-finished movement
      if (zeroing) {
        zeroing = false;                // can't save "arm down" now; next tap starts with taps
        Serial.println(F("# Arm down NOT saved: sensor problem during the save."));
      }
      Serial.print(F("# Sensor? "));
      Serial.print(problem);
      Serial.print(F(" | "));
      printSensorValues();
      if (exerciseCount > 0 && current().mode == SENSOR && state != DONE) {
        warn("Sensor?");
        if (state == RUNNING) Serial.println(F("# Taps count the reps until the sensor is OK."));
      }
    }
  } else {
    badRunning = false;
    if (!goodRunning) { goodRunning = true; goodSince = now; }
    if (!sensorOk && now - goodSince >= GOOD_FOR_MS) {
      sensorOk = true;
      Serial.print(F("# Sensor OK | "));
      printSensorValues();
      if (exerciseCount > 0 && current().mode == SENSOR && state != DONE) {
        showMessage("Sensor OK", MESSAGE_MS);
        playSound(SOUND_CLICK);
      }
    }
  }
}

// Angle between the current gravity direction and the saved "arm down" direction.
float angleFromArmDown() {
  if (!haveRef || totalG < 0.05) return 0;
  float dot = (smoothG[0] * ref[0] + smoothG[1] * ref[1] + smoothG[2] * ref[2]) / totalG;
  if (dot > 1) dot = 1;                 // rounding can push it slightly past 1
  if (dot < -1) dot = -1;
  return acosf(dot) * 57.29578;         // radians to degrees
}

// Arm speed: change in angle over the last 3 readings, using their real times
// (an LCD update can delay a reading a little). When the angle can't be trusted,
// the memory is filled with the current angle so the next reading isn't a "jump".
void updateSpeed(bool valid, uint32_t now) {
  if (!valid) {
    for (uint8_t i = 0; i < 4; i++) { recentAngles[i] = angleDeg; recentAt[i] = now; }
    armSpeed = 0;
    return;
  }
  float oldAngle = recentAngles[recentIndex];       // from 3 readings ago
  uint32_t oldAt = recentAt[recentIndex];
  recentAngles[recentIndex] = angleDeg;
  recentAt[recentIndex] = now;
  recentIndex = (recentIndex + 1) % 4;
  armSpeed = (now > oldAt) ? fabsf(angleDeg - oldAngle) * 1000.0 / (now - oldAt) : 0;
}

// ---------- saving "arm down" ----------
// Starts a fresh half-second of readings.
void restartZeroWindow(uint32_t now) {
  zeroWindowAt = now;
  zeroCount = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    zeroSum[axis] = 0;
    zeroMin[axis] = 99;
    zeroMax[axis] = -99;
  }
}

void startZeroing() {
  zeroing = true;
  zeroStartedAt = millis();
  restartZeroWindow(zeroStartedAt);
}

// Called with every reading while saving. It waits 0.3 s for the tap's shake to
// settle, then needs half a second of a still arm. If the arm moves, that half
// second starts again, for up to 4 s. Uses the unsmoothed readings so the
// smoothing delay doesn't leak the tap movement into the saved position.
void updateZeroing(uint32_t now) {
  if (now - zeroStartedAt < ZERO_SETTLE_MS) {
    restartZeroWindow(now);             // still settling after the tap
    return;
  }
  float spread = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    zeroSum[axis] += rawG[axis];
    zeroMin[axis] = fminf(zeroMin[axis], rawG[axis]);
    zeroMax[axis] = fmaxf(zeroMax[axis], rawG[axis]);
    spread = fmaxf(spread, zeroMax[axis] - zeroMin[axis]);
  }
  zeroCount++;

  if (spread > ZERO_MAX_SPREAD_G) {     // the arm moved
    if (now - zeroStartedAt >= ZERO_GIVE_UP_MS) {
      zeroing = false;
      Serial.print(F("# Arm down NOT saved: the arm kept moving for 4 s (moved "));
      Serial.print(spread, 2);
      Serial.println(F(" g). Let it hang still and tap again."));
      warn("Tap again");
      return;                           // stays READY: the next tap tries again
    }
    restartZeroWindow(now);             // start the still half-second again
    return;
  }
  if (now - zeroWindowAt < ZERO_MS) return;
  zeroing = false;

  float average[3];
  for (uint8_t axis = 0; axis < 3; axis++) average[axis] = zeroSum[axis] / zeroCount;
  float total = lengthOf(average);
  if (total < ZERO_MIN_G || total > ZERO_MAX_G) {
    Serial.print(F("# Arm down NOT saved: total was "));
    Serial.print(total, 2);
    Serial.println(F(" g, expected about 1.00 g when still. Check the sensor wiring."));
    warn("Tap again");
    return;
  }
  for (uint8_t axis = 0; axis < 3; axis++) ref[axis] = average[axis] / total;
  haveRef = true;
  armUp = false;
  startSensorSet();
  playSound(SOUND_START);
  showMessage("Go!", MESSAGE_MS);
  Serial.println(F("# Arm down saved as 0 deg. Set started."));
}

// ---------- rep checker ----------
// Called with every new angle. Returns what happened when a movement ends.
RepResult checkRep(float angle, float speed, uint32_t now) {
  if (!armUp) {
    if (angle < REP_END_DEG) lastBelowEndAt = now;
    if (angle > REP_START_DEG) {        // movement starts
      armUp = true;
      moveStartedAt = lastBelowEndAt;
      movePeak = angle;
      moveTopSpeed = speed;
    }
    return NO_REP;
  }

  if (angle > movePeak) movePeak = angle;
  if (speed > moveTopSpeed) moveTopSpeed = speed;
  if (angle >= REP_END_DEG) return NO_REP;   // still up

  // The arm came back below 25 degrees: the movement is over.
  armUp = false;
  lastBelowEndAt = now;
  lastPeak = movePeak;
  lastTookMs = now - moveStartedAt;
  bool high = (movePeak >= REP_TOP_DEG);
  if (lastTookMs < BUMP_MS)              return REP_BUMP;
  if (high && lastTookMs >= TOO_FAST_MS) return REP_GOOD;
  if (high)                              return REP_TOO_FAST;   // counted, but flagged
  // Under 100 degrees. If the arm was swinging fast, that height can't be trusted:
  // at the top of a fast curl the arm brakes hard and the sensor feels the braking
  // as extra tilt, so the angle reads 20-40 degrees too low. "Slow down" is the
  // right advice either way, so it is "Too fast" (and not counted).
  if (moveTopSpeed > FAST_DEG_PER_S)     return REP_FAST_LOW;
  return REP_TOO_LOW;
}

void printRepLine(const __FlashStringHelper *verdict) {
  if (!PRINT_REPS) return;
  Serial.print(F("# rep check: "));
  Serial.print(verdict);
  Serial.print(F(" peak "));
  Serial.print(lastPeak, 0);
  Serial.print(F(" deg, took "));
  Serial.print(lastTookMs / 1000.0, 1);
  Serial.print(F(" s, top speed "));
  Serial.print(moveTopSpeed, 0);
  Serial.print(F(" deg/s, count "));
  Serial.print(repCount);
  Serial.print('/');
  Serial.println(current().target);
}

// What happens after a movement ends: count it or not, and give feedback.
void handleRep(RepResult result) {
  if (result == NO_REP) return;
  if (result == REP_BUMP) {
    if (PRINT_REPS) Serial.println(F("# rep check: bump ignored"));
    return;
  }
  stats.moves++;
  stats.peakSum += lastPeak;
  stats.msSum += lastTookMs;

  switch (result) {
    case REP_GOOD:
      repCount++;
      stats.sensorReps++;
      playSound(SOUND_TICK);
      showMessage("Good!", GOOD_MS);
      printRepLine(F("good,"));
      break;
    case REP_TOO_FAST:
      repCount++;
      stats.sensorReps++;
      stats.tooFast++;
      warn("Too fast");
      printRepLine(F("TOO FAST (counted),"));
      break;
    case REP_FAST_LOW:
      stats.partial++;
      warn("Too fast");
      printRepLine(F("TOO FAST (not counted, under 100 deg),"));
      break;
    default:                            // REP_TOO_LOW
      stats.partial++;
      warn("Go higher");
      printRepLine(F("GO HIGHER (not counted),"));
      break;
  }
  if (repCount >= current().target) finishSet();
}

// One sensor reading, every 20 ms, whatever the exercise (so the health check
// always knows the sensor's state). Reps are only checked during a sensor set.
void updateSensor() {
  uint32_t now = millis();
  if (now - lastReadAt < READ_EVERY_MS) return;
  lastReadAt = now;
  if (firstReadAt == 0) firstReadAt = now;

  readSensorVolts();
  // Light smoothing: move part of the way (30%) toward each new reading.
  for (uint8_t axis = 0; axis < 3; axis++) {
    if (!haveSmooth) smoothG[axis] = rawG[axis];
    else smoothG[axis] += SMOOTHING * (rawG[axis] - smoothG[axis]);
  }
  haveSmooth = true;
  totalG   = lengthOf(smoothG);
  angleDeg = angleFromArmDown();

  updateHealth(readingProblem(), now);

  if (exerciseCount == 0) return;
  if (zeroing) {
    if (sensorOk) updateZeroing(now);
    updateSpeed(false, now);
    return;
  }
  bool angleValid = inSensorSet() && sensorOk && haveRef;
  updateSpeed(angleValid, now);
  if (angleValid) handleRep(checkRep(angleDeg, armSpeed, now));
  else armUp = false;
}

// ---------- LCD ----------
const uint8_t I2C_DELAY_US = 10;
const uint8_t I2C_RETRIES  = 3;

void sdaLow()  { pinMode(PIN_LCD_SDA, OUTPUT); digitalWrite(PIN_LCD_SDA, LOW); delayMicroseconds(I2C_DELAY_US); }
void sdaHigh() { pinMode(PIN_LCD_SDA, INPUT_PULLUP); delayMicroseconds(I2C_DELAY_US); }
void sclLow()  { digitalWrite(PIN_LCD_SCL, LOW); delayMicroseconds(I2C_DELAY_US); }
void sclHigh() { digitalWrite(PIN_LCD_SCL, HIGH); delayMicroseconds(I2C_DELAY_US); }

void i2cSetup() {
  pinMode(PIN_LCD_SCL, OUTPUT);
  digitalWrite(PIN_LCD_SCL, HIGH);
  sdaHigh();
}

void i2cStart() {
  sdaHigh();
  sclHigh();
  sdaLow();
  sclLow();
}

void i2cStop() {
  sdaLow();
  sclHigh();
  sdaHigh();
}

// Returns true if the chip acknowledges the byte.
bool i2cSendByte(uint8_t value) {
  for (int8_t bit = 7; bit >= 0; bit--) {
    if (value & (1 << bit)) sdaHigh(); else sdaLow();
    sclHigh();
    sclLow();
  }
  sdaHigh();
  sclHigh();
  bool acknowledged = (digitalRead(PIN_LCD_SDA) == LOW);
  sclLow();
  return acknowledged;
}

// Sends two bytes to a chip, retrying if it does not answer.
bool i2cWrite(uint8_t address, uint8_t first, uint8_t second) {
  for (uint8_t attempt = 0; attempt < I2C_RETRIES; attempt++) {
    i2cStart();
    bool ok = i2cSendByte(address << 1) && i2cSendByte(first) && i2cSendByte(second);
    i2cStop();
    if (ok) return true;
    delayMicroseconds(100);
  }
  return false;
}

bool probeI2C(uint8_t address) {
  for (uint8_t attempt = 0; attempt < I2C_RETRIES; attempt++) {
    i2cStart();
    bool ok = i2cSendByte(address << 1);
    i2cStop();
    if (ok) return true;
    delayMicroseconds(100);
  }
  return false;
}

bool lcdCommand(uint8_t command) { return i2cWrite(LCD_TEXT_ADDRESS, 0x80, command); }
bool lcdCharacter(char c)        { return i2cWrite(LCD_TEXT_ADDRESS, 0x40, (uint8_t)c); }

void lcdClear() {
  lcdCommand(0x01);
  delay(3);
}

void lcdPrintLine(uint8_t row, const char *text) {
  lcdCommand(row == 0 ? 0x80 : 0xC0);
  while (*text != 0) lcdCharacter(*text++);
}

// Draws the 7 custom characters into the LCD's character memory (slots 1-7).
// Each is 8 rows of 5 dots; a 1 bit is a lit dot.
void lcdDefineChars() {
  for (uint8_t slot = 1; slot <= 7; slot++) {
    lcdCommand(0x40 | (slot << 3));              // point at that slot's memory
    for (uint8_t row = 0; row < 8; row++) lcdCharacter((char)GLYPHS[slot - 1][row]);
  }
}

void writeLight(uint8_t reg, uint8_t value) { i2cWrite(LCD_LIGHT_ADDRESS, reg, value); }

void setColor(uint8_t red, uint8_t green, uint8_t blue) {
  writeLight(0x04, red);
  writeLight(0x03, green);
  writeLight(0x02, blue);
}

void startLcd() {
  delay(50);
  for (uint8_t i = 0; i < 3; i++) {   // two lines, normal font
    lcdCommand(0x28);
    delay(5);
  }
  lcdCommand(0x0C);                   // display on, no cursor
  lcdClear();
  lcdCommand(0x06);                   // text runs left to right
  lcdDefineChars();

  writeLight(0x00, 0x00);             // wake the backlight chip
  writeLight(0x08, 0xFF);             // LED outputs on
  writeLight(0x01, 0x20);             // backlight chip's output setting
  setColor(0, 90, 255);
  const char splash[] = {' ', ' ', ' ', 'L', 'e', 't', '\'', 's', ' ', 'm', 'o', 'v', 'e', '!', ' ', CH_STAR, 0};
  lcdPrintLine(0, "   Dose Coach   ");
  lcdPrintLine(1, splash);
}

// Sends the LCD its settings, custom characters and backlight setup again,
// without clearing it or waiting. Brings the screen back after an unplug or a
// glitch; the next screen update then redraws every character (about 0.1 s).
void lcdRefresh() {
  for (uint8_t i = 0; i < 3; i++) lcdCommand(0x28);   // two lines, normal font
  lcdCommand(0x0C);                   // display on, no cursor
  lcdCommand(0x06);                   // text runs left to right
  lcdDefineChars();
  writeLight(0x00, 0x00);             // wake the backlight chip
  writeLight(0x08, 0xFF);             // LED outputs on
  writeLight(0x01, 0x20);             // backlight chip's output setting
  forgetScreen();
}

// Forgets what is on the screen, so the next update redraws all of it
// (used after the LCD is started again).
void forgetScreen() {
  memset(shownLine, 0, sizeof shownLine);
  shownColour = -1;
}

void setBacklight() {
  switch (state) {
    case READY:   setColor(0, 90, 255);    break;   // blue
    case RUNNING: setColor(0, 200, 60);    break;   // green
    case PAUSED:  setColor(255, 170, 0);   break;   // amber
    case RESTING: setColor(150, 0, 255);   break;   // purple
    case DONE:    setColor(255, 255, 255); break;   // white
  }
}

void showNoWorkout() {
  static bool shown = false;
  if (!lcdFound || shown) return;
  shown = true;
  setColor(255, 0, 0);
  lcdPrintLine(0, "No exercises    ");
  lcdPrintLine(1, "Check workout.h ");
}

// True for the first few seconds after an exercise is finished: the party screen.
bool celebrating() { return state == DONE && (int32_t)(millis() - celebrateUntil) < 0; }

// Writes only the characters that changed since the last update. Each character
// takes about 1 ms with the hand-made I2C, so this keeps the live bar quick.
// Returns false if the LCD stopped answering.
bool lcdShowRow(uint8_t row, const char *text) {
  int8_t cursor = -1;                           // where the LCD will write next
  for (uint8_t i = 0; i < 16; i++) {
    if (text[i] == shownLine[row][i]) continue;
    if (cursor != (int8_t)i && !lcdCommand((row == 0 ? 0x80 : 0xC0) + i)) return false;
    if (!lcdCharacter(text[i])) return false;
    shownLine[row][i] = text[i];
    cursor = i + 1;
  }
  return true;
}

// Line 1 during a sensor set: a bar of 10 cells that fills as the arm comes up
// (full at 125 degrees), a dotted mark at the 100 degree target, then the angle.
void angleBarLine(char *out) {
  float angle = angleDeg < 0 ? 0 : angleDeg;
  uint8_t columns = (uint8_t)(angle * 50 / BAR_FULL_DEG + 0.5);   // 5 dot-columns per cell
  if (columns > 50) columns = 50;
  for (uint8_t cell = 0; cell < 10; cell++) {
    uint8_t filled = columns > cell * 5 ? columns - cell * 5 : 0;
    if (filled >= 5)      out[cell] = CH_FULL;
    else if (filled > 0)  out[cell] = (char)(CH_BAR1 + filled - 1);   // 1-4 columns lit
    else                  out[cell] = (cell == TARGET_CELL) ? CH_TARGET : ' ';
  }
  snprintf(out + 10, 7, "  %3u", (unsigned)(angle + 0.5));        // fills positions 10-14
  out[15] = CH_DEGREE;
  out[16] = 0;
}

// Fills a screen line with spaces up to the full 16 characters.
void padTo16(char *line) {
  size_t length = strlen(line);
  while (length < 16) line[length++] = ' ';
  line[16] = 0;
}

// Works out both screen lines for the current state.
void buildScreen(char *line1, char *line2, uint32_t now) {
  const Exercise &e = current();
  snprintf(line1, 17, "%-16.16s", e.name);

  // ---- finished: "WELL DONE!" with blinking stars, a row of stars, then a summary
  if (celebrating()) {
    uint32_t age = CELEBRATE_MS - (celebrateUntil - now);       // ms since it started
    char star = ((now / 300) % 2 == 0) ? CH_STAR : ' ';
    const char top[] = {' ', star, ' ', 'W', 'E', 'L', 'L', ' ', 'D', 'O', 'N', 'E', '!', ' ', star, ' ', 0};
    strcpy(line1, top);
    uint8_t stars = (uint8_t)(age / 100);                       // one more star every 0.1 s
    if (stars < 16) {
      for (uint8_t i = 0; i < 16; i++) line2[i] = (i < stars) ? CH_STAR : ' ';
      line2[16] = 0;
    } else if (e.mode == TIMER) {
      snprintf(line2, 17, "%u set%s done     ", e.sets, e.sets == 1 ? "" : "s");
    } else if (e.mode == SENSOR) {
      snprintf(line2, 17, "%u good of %u     ", exGood, exTries);   // out of every rep tried
    } else {
      snprintf(line2, 17, "%u reps done      ", exReps);
    }
    return;
  }

  // ---- whole plan finished: a summary, taking turns with what to do next
  if (state == DONE && workoutComplete()) {
    const char top[] = {'W', 'o', 'r', 'k', 'o', 'u', 't', ' ', 'd', 'o', 'n', 'e', '!', ' ', CH_STAR, ' ', 0};
    strcpy(line1, top);
    uint8_t phases = (wkReps > 0) ? 3 : 2;
    uint8_t phase = (now / PROMPT_MS) % phases;
    if (phases == 2) phase++;                 // no sensor reps: skip the reps line
    if (phase == 0)      snprintf(line2, 17, "Good reps %u/%u", wkGood, wkReps);
    else if (phase == 1) snprintf(line2, 17, "All %u exercises", exerciseCount);
    else                 strcpy(line2, "tap=new round");
    padTo16(line2);
    return;
  }

  // ---- saving "arm down": a progress bar while the arm must stay still
  if (state == READY && zeroing) {
    strcpy(line1, "Hold still...   ");
    uint8_t cells = zeroCount == 0 ? 0 : (uint8_t)((now - zeroWindowAt) * 16 / ZERO_MS);
    if (cells > 16) cells = 16;
    for (uint8_t i = 0; i < 16; i++) line2[i] = (i < cells) ? CH_FULL : '.';
    line2[16] = 0;
    return;
  }

  bool messageShowing = (int32_t)(now - messageUntil) < 0;

  // ---- waiting to start: hints take turns with the normal line
  if (state == READY && !messageShowing) {
    uint8_t phase = (now / PROMPT_MS) % 3;
    if (phase == 1) {
      if (e.mode == SENSOR) {
        const char hint[] = {'A', 'r', 'm', ' ', 's', 't', 'r', 'a', 'i', 'g', 'h', 't', ' ', CH_DOWN, ' ', ' ', 0};
        strcpy(line2, hint);
      } else if (e.mode == REPS) {
        strcpy(line2, "Tap the pad for ");
      } else {
        strcpy(line2, "Tap the pad to  ");
      }
      return;
    }
    if (phase == 2) {
      if (e.mode == SENSOR)    strcpy(line2, "then tap the pad");
      else if (e.mode == REPS) strcpy(line2, "every rep you do");
      else                     strcpy(line2, "start the timer ");
      return;
    }
  }

  // ---- normal line: value on the left, status on the right
  char value[8] = "";
  char status[10] = "";
  if (state == DONE) {
    strcpy(value, "DONE");
    strcpy(status, "tap=next");
  } else if (state == RESTING || state == PAUSED || e.mode == TIMER) {
    uint16_t seconds = (state == READY) ? e.target : secondsLeft();
    snprintf(value, sizeof value, "%02u:%02u", seconds / 60, seconds % 60);
  } else {
    snprintf(value, sizeof value, "%u/%u", repCount, e.target);
  }

  if (state == RESTING) {
    snprintf(line1, 17, setNumber + 1 < 10 ? "Rest, then set %u" : "Rest > set %u", setNumber + 1);
    strcpy(status, "tap=skip");
  } else if (state == PAUSED) {
    strcpy(status, "PAUSED");
  } else if (countingByTaps()) {
    strcpy(status, "Tap reps");               // sensor set, but taps are counting
  } else if (state != DONE) {
    // "Extra 1/2" fits the 9-character status area; from 10 sets on, "Ext 10/12".
    snprintf(status, sizeof status, !extraRound ? "Set %u/%u" : (e.sets < 10 ? "Extra %u/%u" : "Ext %u/%u"),
             setNumber, e.sets);
  }
  if (state == READY && e.mode == SENSOR && !sensorOk) strcpy(status, "Sensor?");  // warn before the tap

  // During a sensor set the top line becomes the live angle bar.
  if (inSensorSet() && sensorOk && haveRef) angleBarLine(line1);

  if (messageShowing) snprintf(status, sizeof status, "%s", message);
  snprintf(line2, 17, "%-7.7s%9.9s", value, status);
}

void updateScreen() {
  if (!lcdFound) return;
  uint32_t now = millis();

  // Backlight: one colour per state, or cycling through colours while celebrating.
  int colour = state;
  if (celebrating()) colour = 10 + (now / 250) % 6;
  if (colour != shownColour) {
    shownColour = colour;
    if (colour >= 10) {
      const uint8_t *c = PARTY_COLOURS[colour - 10];
      setColor(c[0], c[1], c[2]);
    } else {
      setBacklight();
    }
  }

  // Redraw at most every 50 ms; screen writes are slow with hand-made I2C.
  static uint32_t lastDraw = 0;
  if (now - lastDraw < SCREEN_EVERY_MS) return;
  lastDraw = now;

  char line1[17], line2[17];
  buildScreen(line1, line2, now);
  padTo16(line1);
  padTo16(line2);
  if (!lcdShowRow(0, line1) || !lcdShowRow(1, line2)) {
    lcdFound = false;                         // loop() will retry every 3 s
    Serial.println(F("# LCD stopped answering (loose cable?). Retrying every 3 s."));
  }
}

// ---------- Serial Monitor report ----------
void printProbe(const __FlashStringHelper *name, bool answered) {
  Serial.print(name);
  Serial.println(answered ? F("answered") : F("NO ANSWER"));
}

void printProbes() {
  printProbe(F("# LCD text chip 0x3E: "), probeI2C(LCD_TEXT_ADDRESS));
  printProbe(F("# LCD backlight 0x62: "), probeI2C(LCD_LIGHT_ADDRESS));
}

void printReport() {
  Serial.println(F("# --- Dose Coach (Exercise Tracker) ---"));
  printProbes();

  if (!USE_TOUCH) {
    Serial.println(F("# TAP pad: ignored (USE_TOUCH = false). Use button A."));
  } else if (padProbeHigh) {
    Serial.println(F("# TAP pad: pin read HIGH at power-up -> looks UNPLUGGED (or was touched)."));
    Serial.println(F("#   If it really is unplugged, set USE_TOUCH = false."));
  } else {
    Serial.println(F("# TAP pad: looks plugged in."));
  }

  if (exerciseCount > 0) {             // the sensor is only read when there are exercises
    Serial.print(sensorOk ? F("# Wrist sensor: OK | ") : F("# Wrist sensor: NOT OK | "));
    printSensorValues();
    if (!sensorOk) {
      Serial.print(F("#   reason: "));
      Serial.println(lastProblem);
      Serial.println(F("#   Sensor exercises will count taps until it reads OK."));
    }
  }
  Serial.print(F("# Rep rules: start above "));
  Serial.print(REP_START_DEG, 0);
  Serial.print(F(" deg, count at "));
  Serial.print(REP_TOP_DEG, 0);
  Serial.print(F(" deg, end below "));
  Serial.print(REP_END_DEG, 0);
  Serial.print(F(" deg, too fast under "));
  Serial.print(TOO_FAST_MS / 1000.0, 1);
  Serial.println(F(" s"));

  Serial.print(F("# Exercises loaded from workout.h: "));
  Serial.println(exerciseCount);
  for (uint8_t i = 0; i < exerciseCount; i++) {
    const Exercise &e = workout[i];
    Serial.print(F("#   "));
    Serial.print(i + 1);
    Serial.print(F(". "));
    Serial.print(e.name);
    Serial.print(F(": "));
    Serial.print(e.sets);
    Serial.print(F(" x "));
    Serial.print(e.target);
    if (e.mode == TIMER)       Serial.print(F(" s"));
    else if (e.mode == SENSOR) Serial.print(F(" reps (wrist sensor)"));
    else                       Serial.print(F(" reps"));
    Serial.print(F(", rest "));
    Serial.print(e.rest);
    Serial.println(F(" s"));
  }
  if (workoutErrors > 0) readWorkout(false, true);
}

// ---------- main program ----------
void setup() {
  pinMode(PIN_GREEN_LED, OUTPUT);
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_C, INPUT_PULLUP);
  probeTouchPad();                          // leaves the pin as a plain INPUT
  analogReadResolution(12);                 // UNO R4: 0-4095 instead of 0-1023

  Serial.begin(115200);
  workoutErrors = readWorkout(true, false);

  i2cSetup();
  delay(100);                               // let the LCD power up
  lcdFound = probeI2C(LCD_TEXT_ADDRESS);
  if (lcdFound) startLcd();

  playSound(lcdFound ? SOUND_START : SOUND_FINISH);
  uint32_t startedAt = millis();
  while (millis() - startedAt < 1000) {
    updateSound();
    updateLights();
  }
  if (lcdFound) lcdClear();
  forgetScreen();
}

void loop() {
  // LCD check every 3 s. Missing: try again (and report each try). Present: make
  // sure it still answers, and send its settings again, because a loose cable or
  // a quick glitch resets the LCD to a blank, dark screen even if it answers.
  // The resend is skipped during a sensor set, where it would cost readings.
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck >= LCD_RETRY_MS) {
    lastCheck = millis();
    if (!lcdFound) {
      printProbes();
      if (probeI2C(LCD_TEXT_ADDRESS)) {
        lcdFound = true;
        lcdRefresh();
        Serial.println(F("# LCD found again."));
      }
    } else if (!probeI2C(LCD_TEXT_ADDRESS)) {
      lcdFound = false;
      Serial.println(F("# LCD stopped answering (loose cable?). Retrying every 3 s."));
    } else if (!inSensorSet() && !zeroing) {
      lcdRefresh();
    }
  }

  // Once Serial Monitor is open: report, then the CSV header. It waits until the
  // sensor has been read for over 1 s, so the report shows its real state.
  static bool reported = false;
  bool sensorChecked = firstReadAt != 0 && millis() - firstReadAt > BAD_FOR_MS + 200;
  if (!reported && (bool)Serial && (sensorChecked || exerciseCount == 0)) {
    reported = true;
    printReport();
    printLogHeader();
  }

  if (exerciseCount == 0) {
    showNoWorkout();
    updateSound();
    updateLights();
    return;
  }

  // The TAP pad and button A do exactly the same thing. Both are still read
  // during the guard time after a set (to keep their debouncing right), but
  // their taps are ignored then.
  bool tapsAllowed = (int32_t)(millis() - tapsIgnoredUntil) >= 0;
  Press a = readButton(btnA);
  if ((a == SHORT_PRESS || a == LONG_PRESS) && tapsAllowed) handleButtonA();

  if (USE_TOUCH) {
    Press pad = readButton(btnPad);
    if ((pad == SHORT_PRESS || pad == LONG_PRESS) && tapsAllowed) handleButtonA();
  }

  Press b = readButton(btnB);
  if (b != NO_PRESS) handleButtonB(b);

  Press c = readButton(btnC);
  if (c != NO_PRESS) handleButtonC(c);

  updateSensor();
  updateCountdown();
  updateSound();
  updateLights();
  updateScreen();
}
