#ifndef CANBUS_H
#define CANBUS_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"


// =======================================================
// DASH DATA STRUCTURE
// =======================================================

// UNITS ARE IMPERIAL. Each protocol json is responsible for folding the
// conversion into its own signal "scale"/"offset_val", so everything
// downstream can use these values directly with no further maths.
//
//   raw is 0.1 degC        -> "scale":0.18, "offset_val":32
//   raw is degC minus 40   -> "scale":1.8,  "offset_val":-40
//   raw is kPa             -> "scale":0.145038
//   raw is km/h            -> "scale":0.621371
typedef struct{
    float rpm;              // rpm
    float speed;            // MPH
    float coolant_temp;     // degF
    float air_temp;         // degF
    float battery_voltage;  // volts
    float oil_pressure;     // psi
    float air_fuel_ratio;   // lambda / AFR
    float boost;            // psi
    float fuel_comp;        // % ethanol
    float oil_temp;         // degF
    float trans_temp;       // degF
    float fuel_level;       // %
    float fuel_pressure;    // psi
    float gear_sel;         // selector position, protocol specific enum
    float gear_num;         // engaged gear, 1..8
} can_dash_data_t;


// global decoded data
extern volatile can_dash_data_t can_data;



// =======================================================
// API
// =======================================================

void canbus_init(void);
void canbus_task(void *arg);
void process_can_frame(uint32_t id, uint8_t *data);

#endif