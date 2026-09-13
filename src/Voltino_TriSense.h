#ifndef VOLTINO_TRISENSE_H
#define VOLTINO_TRISENSE_H

// Alias for TriSense.h - no API of its own.
//
// It exists so the library ships a header whose filename matches the `name`
// field in library.properties. Arduino Lint rule LS008 looks for exactly that
// (spaces become underscores, so "Voltino TriSense" -> Voltino_TriSense.h) and
// warns when it is missing. The warning never blocked anything, but the name a
// library is published under is awkward to change afterwards, so the cheap fix
// is the one that leaves every existing sketch alone.
//
// Prefer #include <TriSense.h> in your own code: that is what library.properties
// advertises through `includes=`, what the IDE inserts for you, and what every
// example and both documents use. Including this file instead is equivalent.

#include "TriSense.h"

#endif // VOLTINO_TRISENSE_H
