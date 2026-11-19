/*
 * CurlTest - test sketch for the wrist sensor (MMA7361) and the TAP pad
 * Arduino UNO R4 Minima + Grove Base Shield (switch set to 5V)
 *
 * Run this BEFORE changing ExerciseTracker. It only reads the sensor, the
 * TAP pad and button A. It never touches the LCD, buzzer or LEDs, so it is
 * safe to upload with everything else still plugged in.
 *
 * WIRING (the chosen "5V wiring", 5 wires in the cable + 1 jumper)
 *   Sensor 5V  -> Arduino 5V      (the sensor's own regulator makes 3.3 V)
 *   Sensor SL  -> sensor 3V3      (one short female-to-female jumper ON the sensor;
 *                                  Sleep must sit at 3.3 V or the chip outputs nothing)
 *   Sensor X   -> A0
 *   Sensor Y   -> A1
 *   Sensor Z   -> A2
 *   Sensor GND -> GND next to Vin
 *   GS, ST, 0G -> nothing
 *   Grove Touch Sensor (TAP pad) -> Grove socket D6
 *   Button A                     -> D2 and GND (as before)
 *
 * HOW TO USE
 *   1. Open Serial Monitor at 115200 baud. Keep the sensor still for 2 s.
 *      The startup check says whether "total" is close to 1.00 g.
 *      (If it is outside 0.90-1.10 g while still, use the backup 3.3V wiring.)
 *   2. Let the arm hang straight down, then tap the TAP pad (or press A)
 *      with the OTHER hand. Hold still for half a second: that position
 *      becomes 0 degrees and the counts reset.
 *   3. Do curls. One line is printed for every movement:
 *        GOOD       reached 100 deg and took 1.5 s or more   (counted)
 *        TOO FAST   reached 100 deg but took under 1.5 s     (counted, flagged)
 *        TOO FAST   swung faster than 450 deg/s and stayed     (NOT counted: a fast
 *                   under 100 deg                               swing reads too low)
 *        GO HIGHER  never reached 100 deg, at a normal speed   (NOT counted)
 *   4. A status line starting with # is printed every 2 s.
 *
 *   For a graph: set PLOTTER_MODE to true, upload, and open
 *   Tools > Serial Plotter. It draws the angle and the 40 / 100 / 25 lines.
 *
 * HOW THE ANGLE WORKS
 *   When the arm is still, the sensor only feels gravity, which always points
 *   down. "Arm down" saves which way gravity points inside the sensor. Later,
 *   the angle between that saved direction and the current direction is how far
 *   the forearm has swung up from hanging. Twisting the wrist does not change it.
 */

#include <math.h>

// ---------- pins ----------
const uint8_t PIN_BTN_A = 2;                // backup button: pressed = LOW
const uint8_t PIN_TOUCH = 6;                // Grove Touch Sensor: touched = HIGH
const uint8_t PIN_AXIS[3] = {A0, A1, A2};   // sensor X, Y, Z
const char    AXIS_NAME[3] = {'X', 'Y', 'Z'};

// ---------- switches ----------
const bool USE_TOUCH    = true;   // false = ignore the TAP pad. Use this if the pad is
                                  // unplugged: its pin then "floats" and reads random taps.
const bool PLOTTER_MODE = false;  // true = numbers only, for Tools > Serial Plotter

// ---------- rep rules (tune these with this sketch) ----------
const float    REP_START_DEG = 40.0;  // a movement starts when the angle rises above this
const float    REP_TOP_DEG   = 100.0; // it must reach this, or it is "GO HIGHER"
const float    REP_END_DEG   = 25.0;  // it ends when the angle drops below this
const uint16_t TOO_FAST_MS   = 1500;  // shorter than this (start to end) = "TOO FAST"
const uint16_t BUMP_MS       = 300;   // shorter than this = a bump or knock, ignored
const float    FAST_DEG_PER_S = 450;  // arm speed above this = a fast swing (see "TOO FAST" below).
                                      // Nov 18 test: slow full curls 275-330, fast 785, halves 142-362.
// The gap between 40 and 25 degrees stops one wobbly rep from counting twice:
// to count again, the arm has to come all the way back below 25 first.
// A movement's time runs from leaving 25 degrees on the way up to dropping
// back below 25 degrees, so it is close to the whole rep.

// ---------- sensor calibration (measured Nov 6) ----------
const float ZERO_VOLTS[3]  = {1.67, 1.87, 1.47};  // output voltage at 0 g, per axis
const float VOLTS_PER_G[3] = {0.83, 0.80, 0.78};  // change in voltage per 1 g, per axis
const float ADC_VOLTS = 5.0;     // the ADC (analog-to-digital converter) measures against 5 V
const float ADC_STEPS = 4095.0;  // 12-bit reading: 0 ... 4095

// ---------- reading and smoothing (noise handling) ----------
const uint8_t  SAMPLES_PER_AXIS = 16;   // each reading is the average of 16 samples per axis
const uint16_t READ_EVERY_MS    = 20;   // one averaged reading every 20 ms (50 per second)
const float    SMOOTHING        = 0.3;  // 0-1: how much each new reading counts.
                                        // Lower = smoother but slower to follow the arm.

// ---------- sensor health check ----------
const float    MIN_GOOD_VOLTS  = 0.15;  // the chip runs on 3.3 V, so an output outside
const float    MAX_GOOD_VOLTS  = 3.20;  //   0.15-3.20 V means a loose wire or Sleep not at 3.3 V
const float    MIN_GOOD_G      = 0.4;   // a real arm doing curls stays inside 0.4-2.5 g total
const float    MAX_GOOD_G      = 2.5;
const float    SAME_VOLTS      = 0.03;  // all 3 axes within 0.03 V of each other = pins
                                        //   floating (sensor unplugged), not a real reading.
                                        //   A real sensor only does this if held still at one
                                        //   exact tilt; CurlTest shows if that ever happens.
const uint16_t BAD_FOR_MS      = 1000;  // bad for this long  -> "Sensor?"
const uint16_t GOOD_FOR_MS     = 500;   // good for this long -> "Sensor OK"

// ---------- saving "arm down" ----------
const uint16_t ZERO_MS           = 500;   // average the hanging arm over this long
const float    ZERO_MAX_SPREAD_G = 0.15;  // an axis moving more than this = arm not still
const float    ZERO_MIN_G        = 0.8;   // a still arm must read close to 1 g in total
const float    ZERO_MAX_G        = 1.2;

// ---------- timing ----------
const uint16_t STATUS_EVERY_MS  = 2000;
const uint16_t STARTUP_CHECK_MS = 1000;   // keep still this long for the startup check
const uint16_t DEBOUNCE_MS      = 30;     // ignore contact bounce shorter than this
const uint16_t HELD_REPORT_MS   = 5000;   // a press held this long is reported once
const uint8_t  PLOT_EVERY       = 2;      // plotter: send every 2nd reading (25 per second)

// ---------- types ----------
// One input (TAP pad or button), with debouncing and a "held too long" report.
struct Input {
  uint8_t     pin;
  bool        activeHigh;     // true: HIGH means pressed (TAP pad). false: LOW (button)
  const char *name;
  bool        down;           // debounced state
  bool        heldReported;   // the 5 s report was already printed for this press
  uint32_t    changedAt;
  uint32_t    pressedAt;
};

// What the rep checker decided when a movement ended.
enum RepResult : uint8_t { NO_REP, REP_GOOD, REP_TOO_FAST, REP_FAST_LOW, REP_TOO_LOW, REP_BUMP };

// ---------- variables ----------
Input touchPad = {PIN_TOUCH, true,  "TAP pad",  false, false, 0, 0};
Input buttonA  = {PIN_BTN_A, false, "Button A", false, false, 0, 0};
bool  touchProbeHigh = false;      // result of the startup plug-in check

// sensor readings
float volts[3]   = {0, 0, 0};      // latest averaged voltage, per axis
float rawG[3]    = {0, 0, 0};      // latest reading in g (not smoothed)
float smoothG[3] = {0, 0, 0};      // smoothed reading in g
bool  haveSmooth = false;
float totalG     = 0;              // length of the smoothed gravity arrow, in g
float angleDeg   = 0;              // angle from "arm down", in degrees
uint32_t lastReadAt = 0;
uint32_t readingCount = 0;

// sensor health
bool        sensorOk   = false;    // starts false until readings have been good for 0.5 s
bool        badRunning = false;    // currently inside a run of bad readings
bool        goodRunning = false;   // currently inside a run of good readings
uint32_t    badSince   = 0;
uint32_t    goodSince  = 0;
const char *lastProblem = "no readings yet";

// "arm down" reference
bool     haveRef = false;
float    ref[3]  = {0, 0, 0};      // unit-length arrow: gravity direction with arm down
bool     zeroing = false;
uint32_t zeroStartedAt = 0;
uint16_t zeroCount = 0;
float    zeroSum[3], zeroMin[3], zeroMax[3];

// rep checker
bool     armUp = false;            // inside a movement (angle went above 40)
uint32_t moveStartedAt = 0;
uint32_t lastBelowEndAt = 0;       // last time the arm was below 25 deg (rep time starts here)
float    movePeak = 0;
float    moveTopSpeed = 0;         // fastest arm speed in this movement, degrees per second
float    recentAngles[4] = {0, 0, 0, 0};   // last 4 angles, for the speed
uint8_t  recentIndex = 0;
float    armSpeed = 0;             // degrees per second, updated with every reading
float    lastTopSpeed = 0;
uint16_t moveNumber = 0;
uint16_t goodReps = 0, fastReps = 0, fastLowReps = 0, lowReps = 0, bumps = 0;
float    lastPeak = 0;
uint32_t lastTookMs = 0;

// startup check
bool     reported = false;         // header printed once Serial Monitor is open
bool     checkDone = false;
uint32_t checkStartedAt = 0;
uint16_t checkCount = 0;
float    checkSumVolts[3] = {0, 0, 0};

uint32_t lastStatusAt = 0;

// Text lines would confuse the Serial Plotter, so they are only sent in normal mode.
bool talk() { return !PLOTTER_MODE; }

void printStartupCheck();          // written further down

// ---------- inputs ----------
// Returns true once, at the moment a new press starts (after debouncing).
bool newPress(Input &in, uint32_t now) {
  bool level   = (digitalRead(in.pin) == HIGH);
  bool pressed = in.activeHigh ? level : !level;

  if (pressed != in.down && now - in.changedAt >= DEBOUNCE_MS) {
    in.changedAt = now;
    in.down = pressed;
    if (pressed) {
      in.pressedAt = now;
      in.heldReported = false;
      return true;
    }
    if (in.heldReported && talk()) {
      Serial.print(F("# "));
      Serial.print(in.name);
      Serial.println(F(" released"));
    }
  } else if (in.down && !in.heldReported && now - in.pressedAt >= HELD_REPORT_MS) {
    in.heldReported = true;           // report only once per press
    if (talk()) {
      Serial.print(F("# "));
      Serial.print(in.name);
      Serial.println(F(" held for 5 s: stuck, or a hand resting on it?"));
    }
  }
  return false;
}

// Plug-in check for the TAP pad. With the pin's pull-up turned on, an unplugged
// pin reads HIGH. A plugged-in pad that is not being touched drives the pin LOW.
// This only REPORTS the result; whether it is reliable is one of the things to test.
void probeTouchPad() {
  pinMode(PIN_TOUCH, INPUT_PULLUP);
  delay(5);
  touchProbeHigh = (digitalRead(PIN_TOUCH) == HIGH);
  pinMode(PIN_TOUCH, INPUT);          // back to a normal input: the pad drives the pin
}

// ---------- sensor ----------
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

// Prints e.g. "X 1.66 Y 1.88 Z 2.25 V".
void printVolts(const float v[3]) {
  for (uint8_t axis = 0; axis < 3; axis++) {
    Serial.print(AXIS_NAME[axis]);
    Serial.print(' ');
    Serial.print(v[axis], 2);
    Serial.print(axis < 2 ? " " : " V");
  }
}

// Sensor health: one bad reading is ignored. Bad for 1 s -> "Sensor?".
// Good again for 0.5 s -> "Sensor OK". The two different times stop it flickering.
void updateHealth(const char *problem, uint32_t now) {
  if (problem != nullptr) {
    lastProblem = problem;
    goodRunning = false;
    if (!badRunning) { badRunning = true; badSince = now; }
    if (sensorOk && now - badSince >= BAD_FOR_MS) {
      sensorOk = false;
      armUp = false;                  // drop any half-finished movement
      if (zeroing && talk()) Serial.println(F("# Arm down NOT saved: sensor problem during the save."));
      zeroing = false;
      if (talk()) {
        Serial.print(F("# Sensor? "));
        Serial.print(problem);
        Serial.print(F(" | "));
        printVolts(volts);
        Serial.print(F(", total "));
        Serial.print(totalG, 2);
        Serial.println(F(" g"));
      }
    }
  } else {
    badRunning = false;
    if (!goodRunning) { goodRunning = true; goodSince = now; }
    if (!sensorOk && now - goodSince >= GOOD_FOR_MS) {
      sensorOk = true;
      if (talk()) {
        Serial.print(F("# Sensor OK | "));
        printVolts(volts);
        Serial.print(F(", total "));
        Serial.print(totalG, 2);
        Serial.println(F(" g"));
        if (!haveRef) Serial.println(F("# Let the arm hang down, then tap the TAP pad (or press A)."));
      }
    }
  }
}

// Angle between the current gravity direction and the saved "arm down" direction.
float angleFromArmDown() {
  if (!haveRef || totalG < 0.05) return 0;
  float dot = (smoothG[0] * ref[0] + smoothG[1] * ref[1] + smoothG[2] * ref[2]) / totalG;
  if (dot > 1) dot = 1;               // rounding can push it slightly past 1
  if (dot < -1) dot = -1;
  return acosf(dot) * 57.29578;       // radians to degrees
}

// ---------- saving "arm down" ----------
void startZeroing(uint32_t now) {
  if (!sensorOk) {
    if (talk()) {
      Serial.print(F("# Can't save arm down: sensor not OK ("));
      Serial.print(lastProblem);
      Serial.println(F(")"));
    }
    return;
  }
  zeroing = true;
  zeroStartedAt = now;
  zeroCount = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    zeroSum[axis] = 0;
    zeroMin[axis] = 99;
    zeroMax[axis] = -99;
  }
  if (talk()) Serial.println(F("# Hold still... saving arm down"));
}

void resetCounts() {
  armUp = false;
  moveNumber = goodReps = fastReps = fastLowReps = lowReps = bumps = 0;
}

// Called with every reading while saving. Uses the unsmoothed readings so the
// smoothing delay doesn't leak the tap movement into the saved position.
void updateZeroing(uint32_t now) {
  for (uint8_t axis = 0; axis < 3; axis++) {
    zeroSum[axis] += rawG[axis];
    zeroMin[axis] = fminf(zeroMin[axis], rawG[axis]);
    zeroMax[axis] = fmaxf(zeroMax[axis], rawG[axis]);
  }
  zeroCount++;
  if (now - zeroStartedAt < ZERO_MS) return;
  zeroing = false;

  float average[3];
  float spread = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    average[axis] = zeroSum[axis] / zeroCount;
    spread = fmaxf(spread, zeroMax[axis] - zeroMin[axis]);
  }
  float total = lengthOf(average);

  if (spread > ZERO_MAX_SPREAD_G) {
    if (talk()) Serial.println(F("# Arm moved while saving. Hold still and tap again."));
    return;
  }
  if (total < ZERO_MIN_G || total > ZERO_MAX_G) {
    if (talk()) {
      Serial.print(F("# Not saved: total was "));
      Serial.print(total, 2);
      Serial.println(F(" g, expected about 1.00 g when still. Check the wiring."));
    }
    return;
  }
  for (uint8_t axis = 0; axis < 3; axis++) ref[axis] = average[axis] / total;
  haveRef = true;
  resetCounts();
  if (talk()) Serial.println(F("# Arm down saved as 0 deg. Counts reset. Start curling."));
}

// ---------- rep checker ----------
// Called with every new angle. Returns what happened when a movement ends.
// Arm speed: change in angle over the last 3 readings (60 ms), steadier than 1.
// When the angle can't be trusted (sensor problem, no "arm down" yet), the memory
// is filled with the current angle so the next real reading doesn't look like a jump.
void updateSpeed(bool valid) {
  if (!valid) {
    for (uint8_t i = 0; i < 4; i++) recentAngles[i] = angleDeg;
    armSpeed = 0;
    return;
  }
  float oldest = recentAngles[recentIndex];          // angle from 3 readings ago
  recentAngles[recentIndex] = angleDeg;
  recentIndex = (recentIndex + 1) % 4;
  armSpeed = fabsf(angleDeg - oldest) / (3 * READ_EVERY_MS / 1000.0);
}

RepResult checkRep(float angle, float speed, uint32_t now) {
  if (!armUp) {
    if (angle < REP_END_DEG) lastBelowEndAt = now;
    if (angle > REP_START_DEG) {      // movement starts
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
  lastTopSpeed = moveTopSpeed;
  lastTookMs = now - moveStartedAt;
  bool high = (movePeak >= REP_TOP_DEG);
  if (lastTookMs < BUMP_MS)    return REP_BUMP;
  if (high && lastTookMs >= TOO_FAST_MS) return REP_GOOD;
  if (high)                    return REP_TOO_FAST;   // counted, but flagged
  // It did not reach 100 degrees. If the arm was swinging fast, that height can't
  // be trusted: at the top of a fast curl the arm brakes hard, and the sensor feels
  // the braking as extra tilt, so the angle reads 20-40 degrees too low. "Slow down"
  // is the right advice either way, so it is called TOO FAST (and not counted).
  if (moveTopSpeed > FAST_DEG_PER_S) return REP_FAST_LOW;
  return REP_TOO_LOW;
}

void reportRep(RepResult result, uint32_t now) {
  if (result == NO_REP) return;
  if (result == REP_BUMP) {
    bumps++;
    if (talk()) Serial.println(F("# bump ignored (too short to be a movement)"));
    return;
  }
  if (result == REP_GOOD)     goodReps++;
  if (result == REP_TOO_FAST) fastReps++;
  if (result == REP_FAST_LOW) fastLowReps++;
  if (result == REP_TOO_LOW)  lowReps++;
  moveNumber++;
  if (!talk()) return;

  Serial.print(F("MOVE "));
  Serial.print(moveNumber);
  Serial.print(F(": "));
  if (result == REP_GOOD)     Serial.print(F("GOOD      "));
  if (result == REP_TOO_FAST) Serial.print(F("TOO FAST  "));
  if (result == REP_FAST_LOW) Serial.print(F("TOO FAST  "));
  if (result == REP_TOO_LOW)  Serial.print(F("GO HIGHER "));
  Serial.print(F(" peak "));
  Serial.print(lastPeak, 0);
  Serial.print(F(" deg, took "));
  Serial.print(lastTookMs / 1000.0, 1);
  Serial.print(F(" s, top speed "));
  Serial.print(lastTopSpeed, 0);
  Serial.print(F(" deg/s"));
  Serial.print(F(", at t="));
  Serial.print(now / 1000.0, 1);
  Serial.print(F(" s"));
  if (result == REP_TOO_LOW)       Serial.print(F("  (not counted)"));
  else if (result == REP_FAST_LOW) Serial.print(F("  (not counted: under 100 deg)"));
  else                             Serial.print(F("  (counted)"));
  Serial.print(F(" | counted "));
  Serial.print(goodReps + fastReps);
  Serial.print(F(" ("));
  Serial.print(fastReps);
  Serial.print(F(" too fast), not counted: "));
  Serial.print(fastLowReps);
  Serial.print(F(" too fast + low, "));
  Serial.print(lowReps);
  Serial.println(F(" go higher"));
}

// ---------- one sensor reading (every 20 ms) ----------
void handleReading(uint32_t now) {
  readSensorVolts();
  readingCount++;

  // Light smoothing: move part of the way (30%) toward each new reading.
  for (uint8_t axis = 0; axis < 3; axis++) {
    if (!haveSmooth) smoothG[axis] = rawG[axis];
    else smoothG[axis] += SMOOTHING * (rawG[axis] - smoothG[axis]);
  }
  haveSmooth = true;
  totalG   = lengthOf(smoothG);
  angleDeg = angleFromArmDown();

  updateHealth(readingProblem(), now);

  bool angleValid = sensorOk && haveRef && !zeroing;
  updateSpeed(angleValid);
  if (sensorOk && zeroing) updateZeroing(now);
  else if (angleValid) reportRep(checkRep(angleDeg, armSpeed, now), now);

  // Startup check: average the voltages over the first second after the
  // Serial Monitor opens, then say whether total g is close to 1.00.
  if (reported && !checkDone) {
    for (uint8_t axis = 0; axis < 3; axis++) checkSumVolts[axis] += volts[axis];
    checkCount++;
    if (now - checkStartedAt >= STARTUP_CHECK_MS) {
      checkDone = true;
      printStartupCheck();
    }
  }

  if (PLOTTER_MODE && readingCount % PLOT_EVERY == 0) {
    Serial.print(F("angle:"));
    Serial.print(sensorOk ? angleDeg : 0, 1);
    Serial.print(F(",start:"));
    Serial.print(REP_START_DEG, 0);
    Serial.print(F(",top:"));
    Serial.print(REP_TOP_DEG, 0);
    Serial.print(F(",end:"));
    Serial.println(REP_END_DEG, 0);
  }
}

void printStartupCheck() {
  if (!talk()) return;
  float average[3], g[3];
  for (uint8_t axis = 0; axis < 3; axis++) {
    average[axis] = checkSumVolts[axis] / checkCount;        // 1 s average
    g[axis] = (average[axis] - ZERO_VOLTS[axis]) / VOLTS_PER_G[axis];
  }
  float total = lengthOf(g);
  Serial.print(F("# Startup check (sensor still): "));
  printVolts(average);
  Serial.print(F(" -> X "));
  Serial.print(g[0], 2);
  Serial.print(F(" Y "));
  Serial.print(g[1], 2);
  Serial.print(F(" Z "));
  Serial.print(g[2], 2);
  Serial.print(F(" g, total "));
  Serial.print(total, 2);
  Serial.println(F(" g"));
  if (total >= 0.9 && total <= 1.1) {
    Serial.println(F("#   total is inside 0.90-1.10 g: the 5V wiring and calibration are good."));
  } else {
    Serial.println(F("#   total is OUTSIDE 0.90-1.10 g. If the sensor was still, switch to the"));
    Serial.println(F("#   backup 3.3V wiring (Arduino 3.3V, sensor 3V3 and SL in one breadboard row)."));
  }
}

// ---------- status line (every 2 s) ----------
void printStatus(uint32_t now) {
  Serial.print(F("# t="));
  Serial.print(now / 1000);
  Serial.print(F(" s | angle "));
  if (!haveRef)       Serial.print(F("-- (tap to save arm down)"));
  else if (!sensorOk) Serial.print(F("--"));
  else              { Serial.print(angleDeg, 1); Serial.print(F(" deg")); }
  Serial.print(F(" | total "));
  Serial.print(totalG, 2);
  Serial.print(F(" g | "));
  printVolts(volts);
  Serial.print(sensorOk ? F(" | sensor OK") : F(" | SENSOR?"));
  if (!USE_TOUCH)          Serial.print(F(" | pad ignored (USE_TOUCH = false)"));
  else if (touchPad.down)  Serial.print(F(" | pad TOUCHED"));
  else                     Serial.print(F(" | pad not touched"));
  Serial.print(F(" | counted "));
  Serial.print(goodReps + fastReps);
  Serial.print(F(" ("));
  Serial.print(fastReps);
  Serial.print(F(" too fast), not counted: "));
  Serial.print(fastLowReps);
  Serial.print(F(" too fast + low, "));
  Serial.print(lowReps);
  Serial.println(F(" go higher"));
}

void printHeader() {
  Serial.println(F("# --- CurlTest ---"));
  Serial.print(F("# Rules: start above "));
  Serial.print(REP_START_DEG, 0);
  Serial.print(F(" deg, must reach "));
  Serial.print(REP_TOP_DEG, 0);
  Serial.print(F(" deg, ends below "));
  Serial.print(REP_END_DEG, 0);
  Serial.print(F(" deg, too fast under "));
  Serial.print(TOO_FAST_MS / 1000.0, 1);
  Serial.println(F(" s"));
  if (!USE_TOUCH) {
    Serial.println(F("# TAP pad: ignored (USE_TOUCH = false). Use button A."));
  } else if (touchProbeHigh) {
    Serial.println(F("# TAP pad check: pin reads HIGH with pull-up -> pad looks UNPLUGGED"));
    Serial.println(F("#   (or it was being touched at power-up). Plug it into D6, or set USE_TOUCH = false."));
  } else {
    Serial.println(F("# TAP pad check: pad pulls the pin LOW -> pad looks plugged in."));
  }
  Serial.println(F("# Keep the sensor still for the startup check..."));
}

// ---------- main program ----------
void setup() {
  pinMode(PIN_BTN_A, INPUT_PULLUP);
  probeTouchPad();
  analogReadResolution(12);           // UNO R4: 0-4095 instead of 0-1023
  Serial.begin(115200);
}

void loop() {
  uint32_t now = millis();

  // Once Serial Monitor is open: print the header and start the startup check.
  if (!reported && (bool)Serial) {
    reported = true;
    if (talk()) printHeader();
    checkStartedAt = now;
    lastStatusAt = now;
  }

  // A tap on the pad or button A saves "arm down".
  bool tap = false;
  if (USE_TOUCH && newPress(touchPad, now)) {
    tap = true;
    if (talk()) Serial.println(F("# TAP pad tapped"));
  }
  if (newPress(buttonA, now)) {
    tap = true;
    if (talk()) Serial.println(F("# Button A pressed"));
  }
  if (tap) startZeroing(now);

  if (now - lastReadAt >= READ_EVERY_MS) {
    lastReadAt = now;
    handleReading(now);
  }

  if (reported && talk() && now - lastStatusAt >= STATUS_EVERY_MS) {
    lastStatusAt = now;
    printStatus(now);
  }
}
