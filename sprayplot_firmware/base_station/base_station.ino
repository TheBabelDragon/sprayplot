// sprayplot base station
// Boots into BOUNDARY_MAP. Jog the beam, confirm corners, finish the
// envelope, then run SEARCH -> ACQUIRE -> TRACK inside those hard stops.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <math.h>

#include "../shared/protocol.h"

// ---------------------------------------------------------------------------
// Bench knobs — change these after you see real hardware, not before
// ---------------------------------------------------------------------------
static constexpr int SERVO_AZ_PIN = 18;
static constexpr int SERVO_EL_PIN = 19;
static constexpr int LASER_PIN    = 21;

// Usable servo travel. If a mirror slams a stop, shrink these.
static constexpr float SERVO_MIN_DEG = 20.0f;
static constexpr float SERVO_MAX_DEG = 160.0f;

// Amplitude considered "the beam is on the diode".
// Too low: false locks. Too high: never acquires.
static constexpr float LOCK_THRESHOLD = 0.25f;

// Circular dither radius during TRACK.
// Too small: loses lock on fast motion. Too large: visible jitter.
static constexpr float DITHER_DEG = 0.6f;

// Proportional gain on the dither-lock loop.
// Raise until it oscillates, then back off ~1/3.
static constexpr float TRACK_GAIN = 0.35f;

// How long a hit must persist before SEARCH promotes to ACQUIRE/TRACK.
static constexpr uint32_t ACQUIRE_HOLD_MS = 180;

// Dropout tolerated before TRACK declares LOST and falls back to SEARCH.
static constexpr uint32_t LOST_TIMEOUT_MS = 400;

static constexpr float SEARCH_STEP_DEG   = 1.5f;
static constexpr float SEARCH_SETTLE_MS  = 25.0f;
static constexpr float DITHER_HZ         = 8.0f;
static constexpr float STATUS_HZ         = 10.0f;
static constexpr float ALIGN_THRESHOLD   = LOCK_THRESHOLD;

// 50% duty PWM at the shared carrier. Channel 0, 8-bit.
static constexpr int   LASER_LEDC_CH   = 0;
static constexpr int   LASER_LEDC_BITS = 8;
static constexpr int   LASER_DUTY      = 128;

// ---------------------------------------------------------------------------
// Fill this in from a throwaway WiFi.macAddress() sketch on the robot.
// ---------------------------------------------------------------------------
static uint8_t ROBOT_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------------------
static Servo azServo;
static Servo elServo;
static Preferences prefs;

static float az_deg = 90.0f;
static float el_deg = 90.0f;
static float jog_step = 1.0f;

static volatile float last_amp = 0.0f;
static volatile uint32_t last_amp_ms = 0;
static volatile uint32_t last_amp_seq = 0;

static uint8_t state = STATE_BOUNDARY_MAP;

static constexpr int MAX_POINTS = 16;
static float pts_az[MAX_POINTS];
static float pts_el[MAX_POINTS];
static int   n_pts = 0;

static bool  envelope_ready = false;
static float env_az_min = SERVO_MIN_DEG;
static float env_az_max = SERVO_MAX_DEG;
static float env_el_min = SERVO_MIN_DEG;
static float env_el_max = SERVO_MAX_DEG;

static float search_az = 90.0f;
static float search_el = 90.0f;
static int   search_dir = 1;

static uint32_t acquire_since_ms = 0;
static uint32_t last_hit_ms = 0;
static uint32_t last_status_ms = 0;
static uint32_t last_bearing_ms = 0;
static uint32_t bearing_seq = 0;

static float dither_phase = 0.0f;
static float amp_ema = 0.0f;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static const char *stateName(uint8_t s) {
  switch (s) {
    case STATE_BOUNDARY_MAP: return "MAP";
    case STATE_SEARCH:       return "SEARCH";
    case STATE_ACQUIRE:      return "ACQUIRE";
    case STATE_TRACK:        return "TRACK";
    case STATE_LOST:         return "LOST";
    default:                 return "?";
  }
}

static void applyServos() {
  float lo_az = envelope_ready ? env_az_min : SERVO_MIN_DEG;
  float hi_az = envelope_ready ? env_az_max : SERVO_MAX_DEG;
  float lo_el = envelope_ready ? env_el_min : SERVO_MIN_DEG;
  float hi_el = envelope_ready ? env_el_max : SERVO_MAX_DEG;

  // Hard stops always include the mechanical servo limits.
  lo_az = fmaxf(lo_az, SERVO_MIN_DEG);
  hi_az = fminf(hi_az, SERVO_MAX_DEG);
  lo_el = fmaxf(lo_el, SERVO_MIN_DEG);
  hi_el = fminf(hi_el, SERVO_MAX_DEG);

  az_deg = clampf(az_deg, lo_az, hi_az);
  el_deg = clampf(el_deg, lo_el, hi_el);
  azServo.write(az_deg);
  elServo.write(el_deg);
}

static void laserOn() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(LASER_PIN, (uint32_t)CARRIER_HZ, LASER_LEDC_BITS);
  ledcWrite(LASER_PIN, LASER_DUTY);
#else
  ledcSetup(LASER_LEDC_CH, CARRIER_HZ, LASER_LEDC_BITS);
  ledcAttachPin(LASER_PIN, LASER_LEDC_CH);
  ledcWrite(LASER_LEDC_CH, LASER_DUTY);
#endif
}

static void saveEnvelope() {
  prefs.begin("sprayplot", false);
  prefs.putBool("env_ok", true);
  prefs.putFloat("az_min", env_az_min);
  prefs.putFloat("az_max", env_az_max);
  prefs.putFloat("el_min", env_el_min);
  prefs.putFloat("el_max", env_el_max);
  prefs.end();
}

static bool loadEnvelope() {
  prefs.begin("sprayplot", true);
  bool ok = prefs.getBool("env_ok", false);
  if (ok) {
    env_az_min = prefs.getFloat("az_min", SERVO_MIN_DEG);
    env_az_max = prefs.getFloat("az_max", SERVO_MAX_DEG);
    env_el_min = prefs.getFloat("el_min", SERVO_MIN_DEG);
    env_el_max = prefs.getFloat("el_max", SERVO_MAX_DEG);
    envelope_ready = true;
  }
  prefs.end();
  return ok;
}

static void finishMapping() {
  if (n_pts < 3) {
    Serial.println("need at least 3 confirmed points before 'f'");
    return;
  }
  env_az_min = env_az_max = pts_az[0];
  env_el_min = env_el_max = pts_el[0];
  for (int i = 1; i < n_pts; ++i) {
    env_az_min = fminf(env_az_min, pts_az[i]);
    env_az_max = fmaxf(env_az_max, pts_az[i]);
    env_el_min = fminf(env_el_min, pts_el[i]);
    env_el_max = fmaxf(env_el_max, pts_el[i]);
  }
  // Small pad so the lock loop can dither at the edge.
  const float pad = DITHER_DEG + 0.5f;
  env_az_min = clampf(env_az_min - pad, SERVO_MIN_DEG, SERVO_MAX_DEG);
  env_az_max = clampf(env_az_max + pad, SERVO_MIN_DEG, SERVO_MAX_DEG);
  env_el_min = clampf(env_el_min - pad, SERVO_MIN_DEG, SERVO_MAX_DEG);
  env_el_max = clampf(env_el_max + pad, SERVO_MIN_DEG, SERVO_MAX_DEG);
  envelope_ready = true;
  saveEnvelope();
  Serial.printf("envelope az=[%.2f, %.2f] el=[%.2f, %.2f] saved to flash\n",
                env_az_min, env_az_max, env_el_min, env_el_max);
}

static void startSearch() {
  if (!envelope_ready) {
    Serial.println("finish mapping with 'f' before 'g'");
    return;
  }
  search_az = env_az_min;
  search_el = env_el_min;
  search_dir = 1;
  az_deg = search_az;
  el_deg = search_el;
  applyServos();
  acquire_since_ms = 0;
  state = STATE_SEARCH;
  Serial.println("SEARCH");
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
#endif
  if (len < (int)sizeof(AmplitudePacket)) return;
  const AmplitudePacket *p = reinterpret_cast<const AmplitudePacket *>(data);
  if (p->version != PROTO_VERSION || p->type != PKT_AMPLITUDE) return;
  last_amp = p->amplitude;
  last_amp_ms = millis();
  last_amp_seq = p->seq;
}

static void sendBearing() {
  BearingPacket pkt{};
  pkt.version = PROTO_VERSION;
  pkt.type = PKT_BEARING;
  pkt.seq = ++bearing_seq;
  pkt.t_ms = millis();
  pkt.az_deg = az_deg;
  pkt.el_deg = el_deg;
  pkt.amplitude = last_amp;
  pkt.state = state;
  esp_now_send(ROBOT_MAC, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
}

static void printStatus() {
  const bool aligned = last_amp >= ALIGN_THRESHOLD;
  Serial.printf("az=%6.2f  el=%6.2f  step=%.2f  amp=%5.2f  pts=%d  %s%s\n",
                az_deg, el_deg, jog_step, (float)last_amp, n_pts,
                stateName(state),
                aligned ? "  <-- ALIGNED" : "");
}

static void handleMapKey(char c) {
  switch (c) {
    case 'w': el_deg += jog_step; applyServos(); break;
    case 's': el_deg -= jog_step; applyServos(); break;
    case 'a': az_deg -= jog_step; applyServos(); break;
    case 'd': az_deg += jog_step; applyServos(); break;
    case '+':
    case '=':
      jog_step = clampf(jog_step * 2.0f, 0.05f, 10.0f);
      break;
    case '-':
    case '_':
      jog_step = clampf(jog_step * 0.5f, 0.05f, 10.0f);
      break;
    case 'c':
      if (n_pts >= MAX_POINTS) {
        Serial.println("point buffer full");
        break;
      }
      pts_az[n_pts] = az_deg;
      pts_el[n_pts] = el_deg;
      n_pts++;
      Serial.printf("confirmed pt %d  az=%.2f el=%.2f amp=%.2f%s\n",
                    n_pts, az_deg, el_deg, (float)last_amp,
                    last_amp >= ALIGN_THRESHOLD ? "  <-- ALIGNED" : "  (not aligned)");
      break;
    case 'u':
      if (n_pts > 0) {
        n_pts--;
        Serial.printf("undo, pts=%d\n", n_pts);
      }
      break;
    case 'f':
      finishMapping();
      break;
    case 'g':
      startSearch();
      break;
    default:
      break;
  }
}

static void stepSearch() {
  static uint32_t last_move = 0;
  uint32_t now = millis();
  if (now - last_move < (uint32_t)SEARCH_SETTLE_MS) return;
  last_move = now;

  search_az += search_dir * SEARCH_STEP_DEG;
  if (search_az > env_az_max || search_az < env_az_min) {
    search_dir = -search_dir;
    search_az = clampf(search_az, env_az_min, env_az_max);
    search_el += SEARCH_STEP_DEG;
    if (search_el > env_el_max) {
      search_el = env_el_min;
    }
  }
  az_deg = search_az;
  el_deg = search_el;
  applyServos();
}

static void stepTrack(uint32_t now) {
  const float dt = 0.01f;  // loop is not hard real-time; phase is wall-clock based
  (void)dt;
  dither_phase += 2.0f * PI * DITHER_HZ * (1.0f / 200.0f);
  if (dither_phase > 2.0f * PI) dither_phase -= 2.0f * PI;

  const float amp = last_amp;
  if (amp_ema == 0.0f) amp_ema = amp;
  amp_ema = 0.92f * amp_ema + 0.08f * amp;

  const float err = amp - amp_ema;
  az_deg += TRACK_GAIN * err * cosf(dither_phase);
  el_deg += TRACK_GAIN * err * sinf(dither_phase);

  // Command the dithered pointing so the gradient stays observable.
  const float cmd_az = az_deg + DITHER_DEG * cosf(dither_phase);
  const float cmd_el = el_deg + DITHER_DEG * sinf(dither_phase);

  float saved_az = az_deg, saved_el = el_deg;
  az_deg = cmd_az;
  el_deg = cmd_el;
  applyServos();
  az_deg = saved_az;
  el_deg = saved_el;
  (void)now;
}

static void tickTracker() {
  const uint32_t now = millis();
  const float amp = last_amp;
  const bool hit = amp >= LOCK_THRESHOLD;

  if (hit) last_hit_ms = now;

  if (state == STATE_SEARCH) {
    if (hit) {
      if (acquire_since_ms == 0) acquire_since_ms = now;
      state = STATE_ACQUIRE;
    } else {
      stepSearch();
    }
  }

  if (state == STATE_ACQUIRE) {
    if (!hit) {
      acquire_since_ms = 0;
      state = STATE_SEARCH;
    } else if (now - acquire_since_ms >= ACQUIRE_HOLD_MS) {
      amp_ema = amp;
      dither_phase = 0;
      state = STATE_TRACK;
      Serial.println("TRACK");
    }
  }

  if (state == STATE_TRACK) {
    if (now - last_hit_ms > LOST_TIMEOUT_MS) {
      state = STATE_LOST;
      Serial.println("LOST");
    } else {
      stepTrack(now);
    }
  }

  if (state == STATE_LOST) {
    acquire_since_ms = 0;
    state = STATE_SEARCH;
    Serial.println("SEARCH");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("sprayplot base station");
  Serial.printf("this MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("keys: w/s el  a/d az  +/- step  c confirm  u undo  f finish  g go  r remap");

  azServo.setPeriodHertz(50);
  elServo.setPeriodHertz(50);
  azServo.attach(SERVO_AZ_PIN, 500, 2500);
  elServo.attach(SERVO_EL_PIN, 500, 2500);
  applyServos();
  laserOn();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, ROBOT_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("add_peer failed — fill ROBOT_MAC");
  }

  if (loadEnvelope()) {
    Serial.printf("loaded saved envelope: az=[%.2f, %.2f] el=[%.2f, %.2f]\n",
                  env_az_min, env_az_max, env_el_min, env_el_max);
  } else {
    Serial.println("no saved envelope — map corners, then 'f'");
  }
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r') {
      state = STATE_BOUNDARY_MAP;
      Serial.println("BOUNDARY_MAP");
      continue;
    }
    if (state == STATE_BOUNDARY_MAP) {
      handleMapKey(c);
    } else if (c == 'g' && envelope_ready) {
      startSearch();
    }
  }

  if (state != STATE_BOUNDARY_MAP) {
    tickTracker();
  }

  uint32_t now = millis();
  if (now - last_status_ms >= (uint32_t)(1000.0f / STATUS_HZ)) {
    last_status_ms = now;
    printStatus();
  }
  if (now - last_bearing_ms >= 40) {
    last_bearing_ms = now;
    sendBearing();
  }
}
