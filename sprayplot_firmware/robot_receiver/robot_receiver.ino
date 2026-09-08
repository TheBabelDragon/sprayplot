// sprayplot robot receiver
// Goertzel-filters the photodiode ADC at CARRIER_HZ and streams amplitude
// to the base over ESP-NOW. Bearing packets come back the other way.
// fuseWithOdometry() blends rail/hoist/wheel prior with a locked bearing.

#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

#include "../shared/protocol.h"

// ---------------------------------------------------------------------------
static constexpr int   PHOTO_PIN      = 34;     // ADC1
static constexpr int   ADC_BITS       = 12;
static constexpr float SAMPLE_HZ      = 8000.0f;
static constexpr int   GOERTZEL_N     = 200;    // 25 ms at 8 kHz
// Assumes 12-bit ADC, signal roughly mid-scale. Rescale on the bench.
static constexpr float GOERTZEL_NORM  = GOERTZEL_N * 2048.0f;
static constexpr uint32_t SEND_PERIOD_MS = 25;

// Encoder scales — fill in once the trolley drum and wheels exist.
// Until then readMotionPose() reports zeros and the fuse is laser-only
// residual on top of a stationary prior at the rail origin.
static constexpr float MM_PER_RAIL_TICK  = 0.0f;
static constexpr float MM_PER_WINCH_TICK = 0.0f;
static constexpr float MM_PER_WHEEL_TICK = 0.0f;

// Complementary weight on the laser residual while STATE_TRACK.
// 0 = ignore beam, 1 = ignore encoders. Start low.
static constexpr float LASER_WEIGHT = 0.15f;

// Rough mm per degree at a 3 m standoff. Replace with a real
// base-to-wall extrinsic after the first mapped envelope.
static constexpr float MM_PER_DEG_AZ = 52.0f;
static constexpr float MM_PER_DEG_EL = 52.0f;

// ---------------------------------------------------------------------------
// Fill this in from a throwaway WiFi.macAddress() sketch on the base.
// ---------------------------------------------------------------------------
static uint8_t BASE_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static float goertzel_coeff = 0.0f;
static float last_amplitude = 0.0f;
static uint32_t amp_seq = 0;
static uint32_t pose_seq = 0;

static volatile bool have_bearing = false;
static BearingPacket last_bearing{};

static MotionPose motion{};
static PosePacket fused{};

// Hook these to the real counters (TWAI, I2C encoder, GPIO quadrature).
static volatile int32_t rail_ticks  = 0;
static volatile int32_t winch_ticks = 0;
static volatile int32_t wheel_ticks = 0;

static bool have_laser_ref = false;
static float ref_az = 0.0f;
static float ref_el = 0.0f;
static float ref_s  = 0.0f;
static float ref_l  = 0.0f;

// Cheap single-bin detector. Block of N samples, one magnitude out.
static float goertzelBlock() {
  const uint32_t period_us = (uint32_t)(1e6f / SAMPLE_HZ);
  float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
  uint32_t t0 = micros();
  for (int i = 0; i < GOERTZEL_N; ++i) {
    uint32_t due = t0 + period_us * (uint32_t)i;
    while ((int32_t)(micros() - due) < 0) { /* spin */ }
    float x = (float)analogRead(PHOTO_PIN);
    q0 = x + goertzel_coeff * q1 - q2;
    q2 = q1;
    q1 = q0;
  }
  const float real = q1 - q2 * (goertzel_coeff * 0.5f);
  const float imag = q2 * sinf(2.0f * PI * CARRIER_HZ / SAMPLE_HZ);
  const float mag  = sqrtf(real * real + imag * imag);
  return mag / GOERTZEL_NORM;
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
#endif
  if (len < (int)sizeof(BearingPacket)) return;
  const BearingPacket *p = reinterpret_cast<const BearingPacket *>(data);
  if (p->type != PKT_BEARING) return;
  if (p->version < 1 || p->version > PROTO_VERSION) return;
  last_bearing = *p;
  have_bearing = true;
}

static MotionPose readMotionPose() {
  MotionPose p{};
  p.s_mm         = (float)rail_ticks  * MM_PER_RAIL_TICK;
  p.l_mm         = (float)winch_ticks * MM_PER_WINCH_TICK;
  p.wheel_y_mm   = (float)wheel_ticks * MM_PER_WHEEL_TICK;
  p.wheel_x_mm   = 0.0f;
  p.wheel_th_rad = 0.0f;
  p.flags        = POSE_WHEELS_CONTACT;
  return p;
}

// Kinematic prior: trolley station is X, payout is Y-down.
// Wheel Y is a second estimate of payout; blend when both are live.
static void priorFace(const MotionPose &m, float *x, float *y, float *th) {
  *x = m.s_mm + m.wheel_x_mm;
  const bool winch_live = MM_PER_WINCH_TICK != 0.0f;
  const bool wheel_live = MM_PER_WHEEL_TICK != 0.0f;
  if (winch_live && wheel_live) {
    *y = 0.5f * (m.l_mm + m.wheel_y_mm);
  } else if (wheel_live) {
    *y = m.wheel_y_mm;
  } else {
    *y = m.l_mm;
  }
  *th = m.wheel_th_rad;
}

static void fuseWithOdometry(const BearingPacket &bearing) {
  motion = readMotionPose();

  float x_odo = 0.0f, y_odo = 0.0f, th = 0.0f;
  priorFace(motion, &x_odo, &y_odo, &th);

  float x = x_odo;
  float y = y_odo;
  uint8_t flags = motion.flags;

  const bool locked = (bearing.state == STATE_TRACK);
  if (locked) {
    if (!have_laser_ref) {
      ref_az = bearing.az_deg;
      ref_el = bearing.el_deg;
      ref_s  = motion.s_mm;
      ref_l  = motion.l_mm;
      have_laser_ref = true;
    }
    const float x_las = ref_s + (bearing.az_deg - ref_az) * MM_PER_DEG_AZ;
    const float y_las = ref_l + (ref_el - bearing.el_deg) * MM_PER_DEG_EL;
    x = (1.0f - LASER_WEIGHT) * x_odo + LASER_WEIGHT * x_las;
    y = (1.0f - LASER_WEIGHT) * y_odo + LASER_WEIGHT * y_las;
    flags |= POSE_LASER_VALID;
  } else {
    have_laser_ref = false;
  }

  if (flags & POSE_SAFETY_LOCKED) {
    y = y_odo;
  }

  fused.version = PROTO_VERSION;
  fused.type = PKT_POSE;
  fused.seq = ++pose_seq;
  fused.t_ms = millis();
  fused.x_mm = x;
  fused.y_mm = y;
  fused.th_rad = th;
  fused.s_mm = motion.s_mm;
  fused.l_mm = motion.l_mm;
  fused.flags = flags;
  fused.tracker_state = bearing.state;
}

static void sendAmplitude(float amp) {
  AmplitudePacket pkt{};
  pkt.version = PROTO_VERSION;
  pkt.type = PKT_AMPLITUDE;
  pkt.seq = ++amp_seq;
  pkt.t_ms = millis();
  pkt.amplitude = amp;
  esp_now_send(BASE_MAC, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
}

static void sendPose() {
  if (pose_seq == 0) return;
  esp_now_send(BASE_MAC, reinterpret_cast<const uint8_t *>(&fused), sizeof(fused));
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("sprayplot robot receiver");
  Serial.printf("this MAC: %s\n", WiFi.macAddress().c_str());

  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(PHOTO_PIN, ADC_11db);

  const float k = CARRIER_HZ / SAMPLE_HZ;
  goertzel_coeff = 2.0f * cosf(2.0f * PI * k);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BASE_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("add_peer failed — fill BASE_MAC");
  }
}

void loop() {
  last_amplitude = goertzelBlock();
  sendAmplitude(last_amplitude);

  if (have_bearing) {
    BearingPacket snap = last_bearing;
    fuseWithOdometry(snap);
    sendPose();
  }

  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print >= 200) {
    last_print = now;
    Serial.printf("amp=%.3f", last_amplitude);
    if (have_bearing) {
      Serial.printf("  bearing az=%.2f el=%.2f state=%u",
                    last_bearing.az_deg, last_bearing.el_deg,
                    (unsigned)last_bearing.state);
      Serial.printf("  face x=%.0f y=%.0f", fused.x_mm, fused.y_mm);
    }
    Serial.println();
  }
}
