// Finishing line ClearCore firmware — entry point.
//
// All real code lives in the .h/.cpp files beside this file; this is just the
// setup()/loop() shim the Arduino framework calls. See firmware.cpp for the
// behaviour and ../../src/finishing_line/sim/fake_clearcore.py for the
// executable spec this firmware mirrors.
//
// Built with PlatformIO (../../platformio.ini). Not an Arduino sketch — there
// is no matching-name .ino and no sketch-folder convention here.

#include <Arduino.h>
#include "firmware.h"

void setup() { firmwareSetup(); }
void loop() { firmwareLoop(); }
