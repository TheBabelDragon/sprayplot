#pragma once

#include <stdint.h>

// Shared packet structs + constants for the sprayplot laser beacon tracker.
// Included by both sketches as: #include "../shared/protocol.h"

// ---------------------------------------------------------------------------
// Carrier
// ---------------------------------------------------------------------------
// Laser is PWM-modulated at this tone. The robot runs a single-bin Goertzel
// at the same frequency. Pick something the photodiode + TIA can pass and
// that sits well clear of 50/60 Hz and PWM servo noise.
static constexpr float CARRIER_HZ = 1000.0f;

// ESP-NOW payloads are raw packed structs. Keep them small and stable.
static constexpr uint8_t PROTO_VERSION = 1;

enum PacketType : uint8_t {
  PKT_AMPLITUDE = 1,
  PKT_BEARING   = 2,
};

// Robot -> base: demodulated photodiode amplitude at CARRIER_HZ.
struct __attribute__((packed)) AmplitudePacket {
  uint8_t  version;
  uint8_t  type;          // PKT_AMPLITUDE
  uint32_t seq;
  uint32_t t_ms;          // sender millis()
  float    amplitude;     // dimensionless, see Goertzel note in robot sketch
};

// Base -> robot: current beam pointing + tracker state.
struct __attribute__((packed)) BearingPacket {
  uint8_t  version;
  uint8_t  type;          // PKT_BEARING
  uint32_t seq;
  uint32_t t_ms;
  float    az_deg;
  float    el_deg;
  float    amplitude;     // last amplitude the base used
  uint8_t  state;         // TrackerState on the base
};

enum TrackerState : uint8_t {
  STATE_BOUNDARY_MAP = 0,
  STATE_SEARCH       = 1,
  STATE_ACQUIRE      = 2,
  STATE_TRACK        = 3,
  STATE_LOST         = 4,
};
