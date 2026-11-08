/*
 * Handheld Exercise Tracker
 * Arduino UNO R4 Minima + Grove Base Shield (switch set to 5V)
 *
 * WIRING
 *   Grove-LCD RGB Backlight V4.0 -> any socket labelled I2C on the shield
 *   Grove Buzzer                 -> socket D8
 *   Grove LED Socket + red LED   -> socket D5   (warning light)
 *   Button A                     -> pin D2, other side to GND
 *   Button B                     -> pin D3, other side to GND
 *   Button C                     -> pin D4, other side to GND
 *   Green LED                    -> pin D7, through the 220 ohm resistor, to GND
 *   Connect to diagonally opposite legs on each button.
 *
 * CONTROLS
 *   Button A : start / pause / resume a timer, or count one rep
 *              skip rest, or go to the next exercise when DONE
 *   Button B : short press = next exercise, long press = previous exercise
 *   Button C : short press = restart exercise, long press = sound on/off
 *
 * LIGHTS
 *   Green (D7) : flashes with every beep, stays on when the exercise is done
 *   Red   (D5) : flashes when a rest is cut short,
 *                stays on during an extra round of an exercise already finished
 *
 * WORKOUT
 *   The exercise list is in workout.h. Lines with a mistake are skipped, and
 *   Serial Monitor says which line and why.
 *
 * LOG
 *   Every set is printed to Serial Monitor as one CSV row:
 *     time_s,exercise,activity,set,target,done,took_s,note
 *   note is: ok, extra, stopped, left early, or rest cut.
 *   Lines starting with # are status messages, not part of the log.
 *
 * No LCD library is needed. This board's SCL line has no working pull-up, so
 * the Wire library times out. The sketch talks to the LCD itself: it drives SCL
 * both high and low, and it uses the pin's pull-up on SDA.
 *
 * Open Serial Monitor at 115200 baud.
 */

#include "workout.h"

// ---------- pins ----------
const uint8_t PIN_BTN_A     = 2;
const uint8_t PIN_BTN_B     = 3;
const uint8_t PIN_BTN_C     = 4;
const uint8_t PIN_RED_LED   = 5;
const uint8_t PIN_GREEN_LED = 7;
const uint8_t PIN_BUZZER    = 8;
const uint8_t PIN_LCD_SDA   = A4;
const uint8_t PIN_LCD_SCL   = A5;

const uint8_t  LCD_TEXT_ADDRESS  = 0x3E;   // text chip on the Grove LCD
const uint8_t  LCD_LIGHT_ADDRESS = 0x62;   // backlight chip on the V4.0 board
const uint16_t LCD_RETRY_MS    = 3000;
const uint16_t DEBOUNCE_MS     = 30;
const uint16_t LONG_PRESS_MS   = 800;
const uint16_t MESSAGE_MS      = 1200;
const uint16_t WARNING_MS      = 3000;
const bool     PRINT_BUTTONS   = false;    // true = print each button press (wiring test)

const uint8_t MAX_EXERCISES = 30;
const uint8_t NAME_LENGTH   = 16;          // fits one LCD line

// ---------- types ----------
enum Mode : uint8_t { TIMER, REPS };
enum State : uint8_t { READY, RUNNING, PAUSED, RESTING, DONE };
enum Press : uint8_t { NO_PRESS, SHORT_PRESS, LONG_PRESS };

struct Exercise {
  char     name[NAME_LENGTH + 1];
  Mode     mode;     // TIMER or REPS
  uint16_t target;   // seconds for TIMER, reps for REPS
  uint8_t  sets;
  uint16_t rest;     // seconds between sets
};

struct Button {
  uint8_t  pin;
  const char *label;
  bool     down;
  bool     longSent;
  uint32_t changedAt;
  uint32_t pressedAt;
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

Button btnA = {PIN_BTN_A, "A (D2)", false, false, 0, 0};
Button btnB = {PIN_BTN_B, "B (D3)", false, false, 0, 0};
Button btnC = {PIN_BTN_C, "C (D4)", false, false, 0, 0};

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

bool     muted = false;
const char *message = "";
uint32_t messageUntil = 0;

const uint16_t *sound = nullptr;
uint8_t  soundStep = 0;
uint32_t nextSoundStep = 0;
bool     soundOn = false;

const Exercise &current() { return workout[exerciseIndex]; }

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

  if (strlen(mode) != 1) return "mode must be T or C";
  char m = mode[0];
  if (m == 'T' || m == 't') out.mode = TIMER;
  else if (m == 'C' || m == 'c') out.mode = REPS;
  else return "mode must be T or C";

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
    const char *problem = tooLong ? "line is too long" : parseLine(line, exercise, isExercise);

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
  soundOn = on;                                   // green LED follows the sound
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
  Serial.println(F("time_s,exercise,activity,set,target,done,took_s,note"));
}

void logRow(const char *activity, uint8_t set, uint16_t target, uint16_t done,
            uint32_t took, const char *note) {
  Serial.print(millis() / 1000);   Serial.print(',');
  Serial.print(current().name);    Serial.print(',');
  Serial.print(activity);          Serial.print(',');
  Serial.print(set);               Serial.print('/');
  Serial.print(current().sets);    Serial.print(',');
  Serial.print(target);            Serial.print(',');
  Serial.print(done);              Serial.print(',');
  Serial.print(took);              Serial.print(',');
  Serial.println(note);
}

const char *activityName() { return current().mode == TIMER ? "timer" : "reps"; }

// Reps counted, or whole seconds the timer has run, in the current set.
uint16_t doneSoFar() {
  if (current().mode == REPS) return repCount;
  uint32_t total = (uint32_t)current().target * 1000UL;
  uint32_t left = timeLeftMs();
  return (uint16_t)((total - (left < total ? left : total)) / 1000);
}

uint32_t setSeconds() { return (millis() - setStartedAt) / 1000; }

// Logs unfinished work when the participant leaves an exercise early.
void logIfLeftEarly() {
  if (exerciseCount == 0 || state == DONE) return;
  if (state == RUNNING || state == PAUSED) {
    logRow(activityName(), setNumber, current().target, doneSoFar(), setSeconds(), "stopped");
  } else if (state == RESTING) {
    logRow(activityName(), setNumber + 1, current().target, 0, 0, "left early");
  } else if (setNumber > 1) {
    logRow(activityName(), setNumber, current().target, 0, 0, "left early");
  }
}

void prepareSet() {
  repCount = 0;
  state = READY;
}

void goToExercise(int16_t index) {
  logIfLeftEarly();
  if (index < 0) index = exerciseCount - 1;
  if (index >= exerciseCount) index = 0;
  exerciseIndex = (uint8_t)index;
  setNumber = 1;
  extraRound = false;
  prepareSet();
  playSound(SOUND_CLICK);
}

void nextSet() {
  setNumber++;
  prepareSet();
}

void finishSet() {
  logRow(activityName(), setNumber, current().target, doneSoFar(), setSeconds(),
         extraRound ? "extra" : "ok");
  if (setNumber >= current().sets) {
    state = DONE;
    finished[exerciseIndex] = true;
    playSound(SOUND_FINISH);
  } else if (current().rest > 0) {
    state = RESTING;
    startCountdown((uint32_t)current().rest * 1000UL);
    playSound(SOUND_SET);
  } else {
    nextSet();
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

// ---------- buttons ----------
Press readButton(Button &b) {
  bool pressed = (digitalRead(b.pin) == LOW);    // INPUT_PULLUP: LOW means pressed
  uint32_t now = millis();

  if (pressed != b.down && now - b.changedAt >= DEBOUNCE_MS) {
    b.changedAt = now;
    b.down = pressed;
    if (pressed) {
      b.pressedAt = now;
      b.longSent = false;
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
  }
  return NO_PRESS;
}

void handleButtonA() {
  switch (state) {
    case READY:
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
        repCount++;
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
      logRow("rest", setNumber, current().rest, rested, rested, "rest cut");
      nextSet();
      warn("Rest cut");
      break;
    }
    case DONE:
      goToExercise(exerciseIndex + 1);
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
    prepareSet();
    showMessage("Restarted", MESSAGE_MS);
  }
  playSound(SOUND_CLICK);
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

void lcdCommand(uint8_t command) { i2cWrite(LCD_TEXT_ADDRESS, 0x80, command); }
void lcdCharacter(char c)        { i2cWrite(LCD_TEXT_ADDRESS, 0x40, (uint8_t)c); }

void lcdClear() {
  lcdCommand(0x01);
  delay(3);
}

void lcdPrintLine(uint8_t row, const char *text) {
  lcdCommand(row == 0 ? 0x80 : 0xC0);
  while (*text != 0) lcdCharacter(*text++);
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

  writeLight(0x00, 0x00);             // wake the backlight chip
  writeLight(0x08, 0xFF);             // LED outputs on
  writeLight(0x01, 0x20);
  setColor(0, 90, 255);
  lcdPrintLine(0, "Exercise Tracker");
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

void updateScreen() {
  if (!lcdFound) return;

  static int lastState = -1;
  if (state != lastState) {
    lastState = state;
    setBacklight();
  }

  char value[8] = "";
  char status[10] = "";
  uint16_t seconds = 0;

  if (state == DONE) {
    strcpy(value, "DONE");
    strcpy(status, "A: next");
  } else if (state == RESTING || state == PAUSED || current().mode == TIMER) {
    seconds = (state == READY) ? current().target : secondsLeft();
    snprintf(value, sizeof value, "%02u:%02u", seconds / 60, seconds % 60);
  } else {
    snprintf(value, sizeof value, "%u/%u", repCount, current().target);
  }

  if (state == RESTING) {
    snprintf(status, sizeof status, "REST>S%u", setNumber + 1);
  } else if (state == PAUSED) {
    strcpy(status, "PAUSED");
  } else if (state != DONE) {
    snprintf(status, sizeof status, extraRound ? "Extra %u/%u" : "Set %u/%u",
             setNumber, current().sets);
  }

  if ((int32_t)(millis() - messageUntil) < 0) {
    snprintf(status, sizeof status, "%s", message);
  }

  char line1[17], line2[17];
  snprintf(line1, sizeof line1, "%-16.16s", current().name);
  snprintf(line2, sizeof line2, "%-7.7s%9.9s", value, status);

  static char shown1[17] = "", shown2[17] = "";
  if (strcmp(line1, shown1) != 0 || strcmp(line2, shown2) != 0) {
    strcpy(shown1, line1);
    strcpy(shown2, line2);
    lcdPrintLine(0, line1);
    lcdPrintLine(1, line2);
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
  Serial.println(F("# --- Exercise Tracker ---"));
  printProbes();
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
    Serial.print(e.mode == TIMER ? F(" s") : F(" reps"));
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
}

void loop() {
  // Keeps checking until the LCD answers, and reports each try.
  static uint32_t lastCheck = 0;
  if (!lcdFound && millis() - lastCheck >= LCD_RETRY_MS) {
    lastCheck = millis();
    printProbes();
    if (probeI2C(LCD_TEXT_ADDRESS)) {
      lcdFound = true;
      startLcd();
      delay(1000);
      lcdClear();
    }
  }

  // Once Serial Monitor is open: report, then the CSV header.
  static bool reported = false;
  if (!reported && (bool)Serial) {
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

  Press a = readButton(btnA);
  if (a == SHORT_PRESS || a == LONG_PRESS) handleButtonA();

  Press b = readButton(btnB);
  if (b != NO_PRESS) handleButtonB(b);

  Press c = readButton(btnC);
  if (c != NO_PRESS) handleButtonC(c);

  updateCountdown();
  updateSound();
  updateLights();
  updateScreen();
}
