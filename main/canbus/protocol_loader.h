#pragma once

#include <stdint.h>
#include <stdbool.h>

// Must be >= the number of json files in canbus/protocols/ -- the loader
// silently drops any protocol past this limit.
#define MAX_PROTOCOLS 10
// Right-sized against the json actually shipped: haltech is the widest at 21
// frames, ecumasters_black the deepest at 17 signals in one frame. At the old
// 64/16 this table was 285KB of internal DRAM, which overflowed the region and
// meant the CAN build would not link at all -- and MAX_SIGNALS 16 was quietly
// truncating ecumasters' 17th signal ("lc").
#define MAX_FRAMES    32
#define MAX_SIGNALS   20
#define CAN_ID_MAX    2048

typedef enum {
    ENDIAN_BIG,
    ENDIAN_LITTLE
} endian_t;

typedef struct {
    float *target;
    uint8_t offset;      // byte offset, used when bit_len == 0
    uint8_t len;         // 1 or 2 bytes, used when bit_len == 0
    // Bit-level extraction. Set bit_len non-zero to use DBC "start|length"
    // form instead of the byte form above; that is the only way to read a
    // signal that does not begin on a byte boundary, such as a gear selector.
    uint8_t bit_start;   // DBC start bit: MSB for big endian, LSB for little
    uint8_t bit_len;     // 0 = not a bit field
    bool    is_signed;   // two's complement
    float scale;
    float offset_val;
    endian_t endian;
} can_signal_t;

typedef struct {
    uint32_t id;
    int signal_count;
    can_signal_t signals[MAX_SIGNALS];
} can_frame_def_t;

typedef struct {
    char name[32];
    int bitrate;
    int frame_count;
    can_frame_def_t frames[MAX_FRAMES];
} can_protocol_t;

extern can_protocol_t *active_protocol;
extern can_frame_def_t *frame_lookup[CAN_ID_MAX];

void protocol_loader_init(void);
void protocol_detect(uint32_t id);