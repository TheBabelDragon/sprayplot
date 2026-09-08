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
// v2 adds PosePacket; v1 Amplitude/Bearing layouts are unchanged.
static constexpr uint8_t PROTO_VERSION = 2;

enum PacketType : uint8_t {
  PKT_AMPLITUDE = 1,
  PKT_BEARING   = 2,
  PKT_POSE      = 3,
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

// ---------------------------------------------------------------------------
// Motion pose — rail trolley + working-line payout + wall-wheel odom
// ---------------------------------------------------------------------------
// Face frame: origin at the rail's first-section end, +X along the rail,
// +Y down the wall (payout). Wheels roll on the face; they do not lift.
enum PoseFlags : uint8_t {
  POSE_WHEELS_CONTACT = 1 << 0,
  POSE_SAFETY_LOCKED  = 1 << 1,
  POSE_LASER_VALID    = 1 << 2,
};

struct __attribute__((packed)) MotionPose {
  float s_mm;          // trolley station along the rail
  float l_mm;          // working-line payout (0 = snug at the top)
  float wheel_x_mm;    // integrated wheel slip along +X
  float wheel_y_mm;    // integrated wheel roll along +Y
  float wheel_th_rad;  // heading on the face, 0 = rolling +Y
  uint8_t flags;
};

// Robot -> base: fused face pose after complementary blend.
struct __attribute__((packed)) PosePacket {
  uint8_t  version;
  uint8_t  type;          // PKT_POSE
  uint32_t seq;
  uint32_t t_ms;
  float    x_mm;          // fused face X
  float    y_mm;          // fused face Y (down)
  float    th_rad;
  float    s_mm;          // raw trolley
  float    l_mm;          // raw payout
  uint8_t  flags;
  uint8_t  tracker_state; // last bearing.state used in the fuse
};
