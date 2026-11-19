// workout.h - the exercise list, uploaded with the sketch.
// Keep this file in the same folder as ExerciseTracker.ino.
//
// One exercise per line:  name,mode,target,sets,rest
//   name    up to 16 characters (no commas)
//   mode    T = timer
//           C = rep counter (tap the TAP pad or button A for each rep)
//           S = sensor reps (bicep curls with the wrist sensor; reps count themselves)
//   target  seconds for T (max 5999), reps for C and S (max 999)
//   sets    1-99 (optional, default 1)
//   rest    seconds between sets, 0-999 (optional, default 0)
// Lines starting with # are ignored. Start each line at the left edge.
// A line with a mistake is skipped; Serial Monitor (115200 baud) lists it
// with the reason, e.g. "name is longer than 16 characters".
//
// DEMO LIST for the symposium: short targets so a full run takes about 2 minutes.
// The full class plan is kept below, commented out with #.
#pragma once

const char BUILTIN_CSV[] PROGMEM = R"CSV(
# name,mode,target,sets,rest
Bicep Curls,S,5,2,8
Sit to Stand,C,3,2,5
Marching,T,20,1,0

# Full class plan (remove the # to use a line):
# Bicep Curls,S,10,2,30
# Sit to Stand,C,10,2,30
# Wall Push-ups,C,8,2,30
# Heel Raises,C,12,2,20
# Side Leg Raises,C,10,2,20
# Single Leg Stand,T,20,2,15
# Marching,T,45,1,0
)CSV";
