// Minimal Arduino.h stub so OpenFIRE_Square_Advanced.cpp compiles on the host
// for the replay experiment. Only what that translation unit actually uses.
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <initializer_list>
using std::isnan;
using std::isinf;
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif
// Fake clock so OpenFIRE's One Euro filter (which measures dt between CALLS
// via micros()) can be driven deterministically by the simulation.
extern unsigned long g_fake_micros;
inline unsigned long micros() { return g_fake_micros; }
