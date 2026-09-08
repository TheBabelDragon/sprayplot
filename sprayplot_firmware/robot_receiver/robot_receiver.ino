// sprayplot robot receiver
// Goertzel-filters the photodiode ADC at CARRIER_HZ and streams amplitude
// to the base over ESP-NOW. Bearing packets come back the other way.
// fuseWithOdometry() is a stub until the motion board's pose type is known.

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

// ---------------------------------------------------------------------------
// Fill this in from a throwaway WiFi.macAddress() sketch on the base.
// ---------------------------------------------------------------------------
static uint8_t BASE_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static float goertzel_coeff = 0.0f;
static float last_amplitude = 0.0f;
static uint32_t amp_seq = 0;

static volatile bool have_bearing = false;
static BearingPacket last_bearing{};

// Cheap single-bin detector. Block of N samples, one magnitude out.
static float goertzelBlock() {
  const uint32_t period_us = (uint32_t)(1e6f / SAMPLE_HZ);
  float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
  uint32_t t0 = micros();
  for (int i = 0; i < GOERTZEL_N; ++i) {
    // Pace the ADC so the bin lands on CARRIER_HZ.
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
  if (p->version != PROTO_VERSION || p->type != PKT_BEARING) return;
  last_bearing = *p;
  have_bearing = true;
}

// Intentionally empty. Wire this once the motion board's pose representation
// is known (x/y/theta in mm/rad, encoder ticks, whatever it actually is).
static void fuseWithOdometry(const BearingPacket &bearing) {
  (void)bearing;
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
    }
    Serial.println();
  }
}
