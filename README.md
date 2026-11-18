# Dose Coach: exercise tracker (ECE 198)

A handheld tracker for CCCARE exercise classes. It times exercises, counts reps,
and for bicep curls a wrist accelerometer judges each rep as good, too fast or
too low. Every set is logged as one CSV row in Serial Monitor.

Arduino UNO R4 Minima + Seeed Grove Base Shield (switch at 5V).

## Folders

| Folder | What it is |
| --- | --- |
| `ExerciseTracker/` | The tracker: `ExerciseTracker.ino` + `workout.h` (the exercise list) |
| `CurlTest/` | Test sketch for the wrist sensor and the TAP pad (Serial Monitor only) |
| `docs/drawings/` | Pin map, breadboard, sensor cable, base board and rep-rule drawings |

## Wiring

| Part | Connects to |
| --- | --- |
| Grove-LCD RGB Backlight V4.0 | I2C socket |
| Grove Buzzer | D8 socket |
| Grove Touch Sensor (TAP pad, main control) | D6 socket |
| Button A / B / C | D2 / D3 / D4 to GND (breadboard, internal pull-ups) |
| Red LED + 220 ohm | D5 (breadboard) |
| Status LED + 220 ohm | D7 (breadboard) |
| MMA7361 X / Y / Z | A0 / A1 / A2 |
| MMA7361 5V / GND | 5V / GND next to Vin; SL jumpered to the sensor's 3V3 |

See `docs/drawings/04_pin_map.png` for the full map.

## Upload

1. Open `ExerciseTracker/ExerciseTracker.ino` in Arduino IDE (keep `workout.h` in the same folder).
2. Tools > Board > Arduino UNO R4 Minima, pick the port, Upload.
   If it says "No DFU capable USB device", double-tap the reset button and upload again.
3. Open Serial Monitor at 115200 baud for the startup report and the CSV log.

The LCD is driven by the sketch itself (bit-banged I2C), because this board's
SCL line has no working pull-up and the Wire library times out. Don't switch to
an LCD library or the Wire library.

## Log columns

`time_s,exercise,activity,set,target,done,took_s,note,counted_by,partial,too_fast,avg_peak_deg,avg_rep_s`
