#pragma once
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// SEEDING THE ODOMETER
//
// Set this to the vehicle's real mileage. On boot it is written to NVS exactly
// once -- the seed value itself is remembered, so later boots keep whatever
// has accumulated since and do NOT reset back to this number.
//
// To re-seed later, just change this number and reflash; a different value
// counts as a new seed and is applied again.
// ---------------------------------------------------------------------------
#define ODO_SEED_MILES 79645.0

void odometer_init(void);

// Preferred: distance from speed over time. Keeps the sub-meter remainder, so
// slow driving accumulates correctly instead of being truncated away.
void odometer_add_miles(double miles);

// Whole meters. Retained for callers that already work in metres.
void odometer_add_meters(uint32_t meters);

void odometer_periodic_save(void);
void odometer_force_save(void);

uint64_t odometer_get_meters(void);
double odometer_get_miles(void);
