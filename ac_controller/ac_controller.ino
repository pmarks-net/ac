#include <IRsend.h>
#include <WiFi.h>
#include <cstdio>

#define ARRAY_SIZE(array) ((sizeof(array))/(sizeof(array[0])))

const char WIFI_SSID[] = "redacted";
const char WIFI_PASS[] = "redacted";
const char SERVER_IP[] = "192.168.4.1";
const int SERVER_PORT = 4533;

const int IR_LED_PIN = 23;
const int IR_KHZ = 38;

// AP14001HS IR codes
const uint16_t IR_0 = 625;   // stable range is 590-700
const uint16_t IR_1 = 1500;  // stable range is 1130-1900ish
// IR_STOP is a delay to allow playing codes back-to-back.
// Test procedure:
// - Go to COOL and set temperature to 88.
// - Send "88->80->80->88", pause 1 second, and loop forever.
// With IR_STOP=1100, this loses sync after about a minute.
// With IR_STOP=1500, this ran for ~45 minutes (several hundred
// cycles) at which point it missed an entire sequence of codes,
// which seems like a different kind of bug.
const uint16_t IR_STOP = IR_1;
const uint16_t IR_FAN_SPEED[] = {
  IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0,
  IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1,
  IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_0, IR_0, IR_1, IR_0, IR_0, IR_0,
  IR_1, IR_0, IR_1, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_0,
  IR_0, IR_1, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0,
  IR_STOP
};
const uint16_t IR_POWER[] = {
  IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0,
  IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1,
  IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_0, IR_0, IR_1, IR_0, IR_0, IR_0,
  IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_0,
  IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0,
  IR_STOP
};
const uint16_t IR_MODE[] = {
  IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0,
  IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1,
  IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_0, IR_0, IR_0, IR_0,
  IR_1, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1,
  IR_0, IR_1, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0,
  IR_STOP
};
const uint16_t IR_UP[] = {
  IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0,
  IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1,
  IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_0, IR_0, IR_1, IR_0, IR_0, IR_0,
  IR_1, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_0,
  IR_0, IR_1, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0,
  IR_STOP
};
const uint16_t IR_DOWN[] = {
  IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1, IR_0,
  IR_0, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1,
  IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0, IR_0, IR_0, IR_0, IR_0,
  IR_0, IR_0, IR_1, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_0, IR_1,
  IR_0, IR_1, IR_0, IR_1, IR_0, IR_0, IR_0, IR_1, IR_0, IR_1, IR_0, IR_1, IR_0,
  IR_STOP
};

// It seems that I can't use "enum" from an .ino file
#define ACMode int
const ACMode MODE_COOL = 0;
const ACMode MODE_DEHU = 1;  // Not used much.
const ACMode MODE_FAN = 2;
const ACMode MODE_HEAT = 3;
// The A/C cycles through this many modes when we send irMode()
const ACMode MODE_CYCLES = 4;

// In some contexts, we can treat "OFF" as an extra virtual mode.
const ACMode MODE_OFF = 4;
const ACMode MODE_COUNT = 5;

// More virtual modes; printable but not counted.
const ACMode MODE_INVALID = 5;
const ACMode MODE_COOL_IDLE = 6;
const ACMode MODE_HEAT_IDLE = 7;

const char* const MODE_NAMES[] = {
  "COOL", "DEHU", "FAN", "HEAT", "OFF", "INVALID", "COOL_IDLE", "HEAT_IDLE"
};

struct ACState {
  // When cool_temp is negative, we synchronize the temperature range
  // to 18..27.  For ironclad temperature control, we could reset this
  // whenever resyncAndSetActiveMode() runs, but it's actually useful
  // to let it float, so the user can manually raise the set point
  // (up to 23..32) so smaller rooms don't get too cold.
  int heat_temp = -1;
  int cool_temp = -1;

  ACMode active_mode = MODE_INVALID;
  ACMode memory_mode = MODE_INVALID;
};

template<typename ... Args>
void serialPrintf(const char* format, Args ... args) {
  char buf[256] = "";
  snprintf(buf, sizeof(buf), format, args ...);
  Serial.print(buf);
}

template<typename ... Args>
[[noreturn]] void panic(const char* format, Args ... args) {
  serialPrintf(format, args ...);
  while (true) {
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
  }
}

// Longer than any other delays.
const unsigned long FOREVER_MS = 86400*1000;

struct Delay {
  bool en = false;
  unsigned long start_time;
  int duration = -1;
  
  void start(int duration0 = -1) {
    duration = duration0;
    start_time = millis();
    en = true;
  }

  bool justFired() {
    if (duration < 0) panic("undefined duration!\n");
    if (en && elapsed() >= duration) { 
      en = false;
      return true; 
    }
    return false;
  }

  int elapsed() {
    if (!en) return FOREVER_MS;
    // Keep the start_time fresh to avoid wraparound.
    const unsigned long now = millis();
    if (now - start_time > FOREVER_MS) {
      start_time = now - FOREVER_MS;
    }
    return now - start_time;
  }
};

// Do a full mode sync every 3 hours.
const int RESYNC_MS = 3*3600*1000;

// AP14001HS temperature limits, in F and C.
// We only support celsius because:
// - Temperature changes require fewer clicks
// - Cycling through modes will corrupt fahrenheit temperatures,
//   reducing the value by 0-2 degrees.  I'm guessing the device
//   is using C internally, with lossy F->C->F conversion.
//
// Alas, if you remove power from the AP14001HS while it's running,
// it reboots to fahrenheit, with no way to recover automatically,
// The best we can do is "fail safe" by shifting the ranges from
// 18..27C -> 75..84F (cool), and 18..25C -> 61..70F (heat).
//
// Sync procedure: if you see "F", press in the temperature dial
//                 and power cycle the controller.
const int COOL_MIN_F = 61;
const int COOL_MAX_F = 89;
const int COOL_MIN_C = 16;
const int COOL_MAX_C = 32;
const int HEAT_MIN_F = 61;
const int HEAT_MAX_F = 77;
const int HEAT_MIN_C = 16;
const int HEAT_MAX_C = 25;

// Temperatures below/above a reasonable human's thermostat settings,
// so we can send fewer clicks in the common case.
// (For heat, just use Min/Max, because the range is smaller.)
const int COOL_RUN_TEMP = 18;
const int COOL_IDLE_TEMP = 27;

int comparableSeq(uint16_t sent_qseq, uint16_t qseq, int rseq) {
  if (0 <= rseq && rseq <= 0xf) {
    if (qseq == sent_qseq - 1) {
      return 0x10 | rseq;
    } else if (qseq == sent_qseq) {
      return 0x20 | rseq;
    }
  }
  return 0x00;
}

// I'm using an INL-3APD80 photodiode in photovoltaic mode,
// "PIN --->|--- GND", in parallel with a 10nF capacitor.
const int PHOTO_PIN = 33;
// Time between analog reads; lag occurs around <= 25us.
const int READ_US = 50;
// 50us*2000 = 100ms pre-read, because the act of reading
// seems to disturb the measured value.
const int SKIP_COUNT = 2000;
// 50us*2000 = 100ms read, to average out the noise.
// Raising this to 20000 (1000ms) makes it possible to run
// without the 10nF capacitor.
const int READ_COUNT = 2000;

int filteredRead() {
  int sum = 0;
  const unsigned long start = micros();
  for (int i = 0; i < SKIP_COUNT + READ_COUNT; i++) {
    const int val = analogRead(PHOTO_PIN);
    if (i >= SKIP_COUNT) {
      sum += val;
    }
    const int target_us = ((i + 1) * READ_US) + random(READ_US / 5);
    const int wait = target_us - (micros() - start);
    if (wait > 0) {
      delayMicroseconds(wait);
    }
  }
  return sum;
}

void cycleMode(ACMode* m) {
  if (!(MODE_COOL <= *m && *m < MODE_CYCLES)) {
    panic("Can't cycle from mode %d\n", *m);
  }
  *m = (*m + 1) % MODE_CYCLES;
}

// This defines the minimum separation between light levels.
const float STEP_FACTOR = 1.05;
int stepUp(int val) {
  return val * STEP_FACTOR;
}
int stepDn(int val) {
  return val * (1 / STEP_FACTOR);
}

IRsend irsend(IR_LED_PIN);

void irPower() {
  irsend.sendRaw(IR_POWER, ARRAY_SIZE(IR_POWER), IR_KHZ);
}

void irMode() {
  irsend.sendRaw(IR_MODE, ARRAY_SIZE(IR_MODE), IR_KHZ);
}

int irTemp(int old_temp, int new_temp, bool* first=nullptr) {
  serialPrintf("Changing temp: %d -> %d\n", old_temp, new_temp);
  bool first_default = true;
  if (first == nullptr) {
    first = &first_default;
  }
  int t = old_temp;
  while (t < new_temp) {
    irsend.sendRaw(IR_UP, ARRAY_SIZE(IR_UP), IR_KHZ);
    if (*first) {
      *first = false;
    } else {
      t++;
    }
  }
  while (t > new_temp) {
    irsend.sendRaw(IR_DOWN, ARRAY_SIZE(IR_DOWN), IR_KHZ);
    if (*first) {
      *first = false;
    } else {
      t--;
    }
  }
  return new_temp;  // for convenience
}

// When this returns, the A/C will be off,
// and we'll know the luma value for "off".
int measureOffLuma() {
  const int luma1 = filteredRead();
  serialPrintf("Before POWER: luma=%d\n", luma1);
  irPower();  // mystery on/off
  const int luma2 = filteredRead();
  if (luma2 > stepUp(luma1)) {
    serialPrintf("After POWER: luma=%d (on!)\n", luma2);
    irPower();
    return luma1;
  } else if (luma2 < stepDn(luma1)) {
    serialPrintf("After POWER: luma=%d (off!)\n", luma2);
    return luma2;
  } else {
    serialPrintf("After POWER: luma=%d (no effect!)\n", luma2);
    irPower();  // Better to shout an even number of POWERs into the void.
    panic("POWER didn't seem to do anything.\n");
  }
}

int maxIndex(const std::vector<int>& vals) {
  int max_val = INT_MIN;
  int max_index = 0;
  for (int i = 0; i < vals.size(); i++) {
    if (vals[i] > max_val) {
      max_val = vals[i];
      max_index = i;
    }
  }
  return max_index;
}

int lumaOrder(ACMode mode) {
  switch (mode) {
    case MODE_OFF:  return 1;
    case MODE_HEAT: return 2;
    case MODE_FAN:  return 3;
    case MODE_COOL: return 4;
    default: panic("Mode %d has no luma order\n", mode);
  }
}

bool orderOk(ACMode old_mode, int old_luma, ACMode new_mode, int new_luma) {
  if (old_mode == new_mode) {
    panic("orderOk requires different modes\n");
  }
  const bool old_is_lo = lumaOrder(old_mode) < lumaOrder(new_mode);
  const ACMode lo_mode = old_is_lo ? old_mode : new_mode;
  const ACMode hi_mode = old_is_lo ? new_mode : old_mode;
  const int lo_luma = old_is_lo ? old_luma : new_luma;
  const int hi_luma = old_is_lo ? new_luma : old_luma;
  if (!(lo_luma < hi_luma)) {
    serialPrintf("Modes %s (luma=%d) and %s (luma=%d) out of order\n",
                 MODE_NAMES[lo_mode], lo_luma, MODE_NAMES[hi_mode], hi_luma);
    return false;
  }
  if (!(stepUp(lo_luma) < hi_luma)) {
    serialPrintf("Modes %s (luma=%d) and %s (luma=%d) are too close\n",
                 MODE_NAMES[lo_mode], lo_luma, MODE_NAMES[hi_mode], hi_luma);
    return false;
  }
  return true;
}

// XXX maybe force a resync instead??
void orderOkOrPanic(ACMode old_mode, int old_luma, ACMode new_mode, int new_luma) {
  if (!orderOk(old_mode, old_luma, new_mode, new_luma)) {
    panic("Transition from %s -> %s violated luma order!\n",
          MODE_NAMES[old_mode], MODE_NAMES[new_mode]);
  }
}

// This must be called immediately after measureOffLuma().
// When we return, the A/C will be in the mode select menu for
// the next several seconds, pointing at the returned mode.
ACMode synchronizeMode(int off_luma) {
  std::vector<int> lumas;
  while (lumas.size() < MODE_CYCLES) {
    irMode();
    const int l = filteredRead();
    serialPrintf("Sampled luma=%d for mystery mode %d\n", l, lumas.size());
    lumas.push_back(l);
  }

  // "Cool" is the brightest value.
  const int cool_idx = maxIndex(lumas);

  const std::vector<int> mode_lumas = {
    lumas[cool_idx],
    lumas[(cool_idx + 1) % MODE_CYCLES],
    lumas[(cool_idx + 2) % MODE_CYCLES],
    lumas[(cool_idx + 3) % MODE_CYCLES],
    off_luma
  };

  bool all_ok = true;
  for (int i = 0; i < MODE_COUNT; i++) {
    serialPrintf("Mode %4s has luma=%d\n", MODE_NAMES[i], mode_lumas[i]);
    for (int j = i+1; j < MODE_COUNT; j++) {
      if (i != MODE_DEHU && j != MODE_DEHU) {
        all_ok &= orderOk(i, mode_lumas[i], j, mode_lumas[j]);
      }
    }
  }
  if (!all_ok) {
    // Try to restore the initial mode before dying.
    irMode();  
    panic("Mode order seems wrong -- check photodiode polarity?\n");
  }

  // Which mode was lit initially?
  const ACMode init_mode = (MODE_CYCLES - cool_idx) % MODE_CYCLES;

  // Which mode is lit now?
  const ACMode out = (init_mode + MODE_CYCLES - 1) % MODE_CYCLES;

  return out;
}

// Caller should update resync_d.
void resyncAndSetActiveMode(ACMode new_mode, ACState* acs) {
  bool leave_power_off = false;
  switch (new_mode) {
    case MODE_OFF:
      // To sync into OFF, just turn the fan on+off.
      new_mode = MODE_FAN;
      leave_power_off = true;
      break;
    case MODE_HEAT:
    case MODE_COOL:
    case MODE_FAN:
      break;
    default:
      panic("mode %d not supported here\n", new_mode);
  }
  const int off_luma = measureOffLuma();
  acs->active_mode = synchronizeMode(off_luma);
  while (acs->active_mode != new_mode) {
    irMode();
    cycleMode(&acs->active_mode);
  }

  applyActiveMode(true, acs);
  
  if (leave_power_off) {
    irPower();
    acs->active_mode = MODE_OFF;
  } else {
    const int on_luma = filteredRead();
    orderOkOrPanic(MODE_OFF, off_luma, new_mode, on_luma);
  }
}

void applyActiveMode(bool in_menu, ACState* acs) {
  if (in_menu) {
    // On the AP14001HS, when I cycle through modes, stop on FAN,
    // and then press POWER (1 or 3 times), it sometimes starts
    // either with the fan on low, or the compressor
    // (but not the exhaust) running.  I've been able to reproduce
    // this with delay(<=800), but not with delay(>=900),
    // so 1000 seems like a safe value.
    delay(1000);
  }
  irPower();
  acs->memory_mode = acs->active_mode;

  bool first = true;
  switch (acs->active_mode) {
    case MODE_HEAT:
      if (acs->heat_temp < 0) {
        // Clamp to min F, but hope that we're in C mode.
        irTemp(HEAT_MAX_F, HEAT_MIN_F, &first);
        acs->heat_temp = HEAT_MIN_C;
      }
      acs->heat_temp = irTemp(acs->heat_temp, HEAT_MAX_C, &first);
      break;
    case MODE_COOL:
      if (acs->cool_temp < 0) {
        // Clamp to max F, but hope that we're in C mode.
        irTemp(COOL_MIN_F, COOL_MAX_F, &first);
        acs->cool_temp = COOL_MAX_C;
      }
      acs->cool_temp = irTemp(acs->cool_temp, COOL_RUN_TEMP, &first);
      break;
    case MODE_FAN:
      break;
    default:
      panic("mode %d not supported here\n", acs->active_mode);
  }
}

void powerOff(ACState* acs) {
  const int on_luma = filteredRead();
  irPower();
  const int off_luma = filteredRead();
  orderOkOrPanic(acs->memory_mode, on_luma, MODE_OFF, off_luma);
  acs->active_mode = MODE_OFF;
}

struct ProtocolState {
  WiFiUDP udp;

  // Sequencing algorithm:
  // - Every couple seconds, the client sends a query (Q) containing
  //   a randomly-initialized query sequence number (qseq).
  // - The server immediately responds with a response (R) containing
  //   "R qseq rseq mode", where qseq is copied from the query, and
  //   rseq is 0.
  // - When the server wants to fast-push a new mode, it sends another
  //   packet with rseq incremented.
  // - The client will only accept responses with the current or
  //   previous qseq, and (qseq, rseq) must grow lexicographically.
  uint16_t sent_qseq;

  // Initialized after connecting to WiFi.
  uint16_t recvd_qseq;
  int recvd_rseq;
};

void sendUDP(struct ProtocolState* prs) {
  char buf[16];
  const int buf_len = sprintf(buf, "Q %04x", ++prs->sent_qseq);
  prs->udp.beginPacket(SERVER_IP, SERVER_PORT);
  prs->udp.write(reinterpret_cast<const uint8_t*>(buf), buf_len);
  prs->udp.endPacket();
}

ACMode receiveUDP(struct ProtocolState* prs) {
  ACMode out = MODE_INVALID;
  while (prs->udp.parsePacket() > 0) {
    char buf[32];
    const int buf_len = prs->udp.read(buf, ARRAY_SIZE(buf)-1);
    if (buf_len < 0) continue;
    buf[buf_len] = '\0';
    char buf_type;
    uint16_t buf_qseq;
    int buf_rseq;
    char buf_mode[ARRAY_SIZE(buf)];
    if (std::sscanf(buf, "%c %04" SCNx16 " %1x %s",
                    &buf_type, &buf_qseq, &buf_rseq, buf_mode) != 4) {
      serialPrintf("packet malformed\n");
      continue;
    }
    if (buf_type != 'R') {
      serialPrintf("packet type '%c'\n");
      continue;
    }
    const int last_cmp_seq = comparableSeq(prs->sent_qseq, prs->recvd_qseq, prs->recvd_rseq);
    const int this_cmp_seq = comparableSeq(prs->sent_qseq, buf_qseq, buf_rseq);
    if (!(this_cmp_seq > last_cmp_seq)) {
      serialPrintf("packet late or misordered\n");
      continue;
    }
    if (buf_qseq == prs->sent_qseq) {
      const int missed_responses = ((buf_qseq - prs->recvd_qseq) & 0xFFFF) - 1;
      if (missed_responses > 0) {
        serialPrintf("Recovered after %d missed responses\n", missed_responses);
      }
    }
    prs->recvd_qseq = buf_qseq;
    prs->recvd_rseq = buf_rseq;
    
    // TODO: Read entire response.
    switch (buf_mode[0]) {
      case 'H':
        out = MODE_HEAT;
        break;
      case 'C':
        out = MODE_COOL;
        break;
      case 'F':
        out = MODE_FAN;
        break;
      case 'O':
        out = MODE_OFF;
        break;
    }
  }
  return out;
}

void setup() {
  Serial.begin(115200);

  // Underclocking to 80 MHz breaks IR transmission.
  // 160 MHz isn't obviously broken, but it's not worth the risk.
  serialPrintf("CPU: %d MHz\n", getCpuFrequencyMhz());
  
  analogSetAttenuation(ADC_0db);
  randomSeed(analogRead(PHOTO_PIN));
  pinMode(2, OUTPUT);
  irsend.begin();
}

void loop() {
  static ACState acs;
  static ProtocolState prs;
  static Delay resync_d;
  static Delay active_mode_d;
  static Delay compressor_idle_d;
  static Delay wifi_connecting_d;
  static Delay wifi_reconnect_d;
  static Delay next_udp_d;
  static int last_compressor_idle_ms = FOREVER_MS;
  static ACMode want_mode = MODE_INVALID;

  static bool init = true;
  if (init) {
    wifi_reconnect_d.start(0);
    active_mode_d.start();  // Hold INVALID for a bit.
    init = false;
  }

  // Use lambdas to avoid passing lots of variables.
  auto transition_hold_time = [&]() {
    if (acs.active_mode == want_mode) {
      return 0;
    }
    switch (acs.active_mode) {
      case MODE_INVALID:
        // After a power failure, wait 5 seconds for the A/C to settle.
        return 5*1000;
      case MODE_HEAT_IDLE:
      case MODE_COOL_IDLE: {
        // If the compressor was idle for less than 10 minutes
        // during the last cycle, then try to cover the entire
        // idle gap with fan noise.
        // (I'm not sure if this is useful in heat mode.)
        static const int IDLE_THRESHOLD_MS = 600*1000;
        if (want_mode == MODE_OFF &&
            last_compressor_idle_ms < IDLE_THRESHOLD_MS) {
          return IDLE_THRESHOLD_MS * 3 / 2;  // 150%
        }
      }  // fallthrough
      case MODE_HEAT:
      case MODE_COOL:
        // Keep the compressor on/off for 3 minutes.
        // The bare minimum is ~20s, to exit the temperature menu.
        return 180*1000;
      default:
        return 0;
    }
  };
  auto print_next_transition = [&]() {
    const int t = transition_hold_time() - active_mode_d.elapsed();
    if (t > 0) {
      serialPrintf("Transition from %s -> %s in %d ms\n",
                   MODE_NAMES[acs.active_mode], MODE_NAMES[want_mode], t);
    }
  };

  resync_d.elapsed();  // Don't let this get too old.
  
  if (wifi_reconnect_d.justFired()) {
    serialPrintf("Connecting to %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifi_connecting_d.start(10000);
  }
  if (wifi_connecting_d.justFired()) {
    WiFi.disconnect();
    wifi_reconnect_d.start(10000);
    serialPrintf("WiFi's dead; reconnect in %d ms...\n", wifi_reconnect_d.duration);
    want_mode = MODE_OFF;
  }
  if (wifi_reconnect_d.en) {
    // ignoring WiFi.
  } else if (WiFi.status() == WL_CONNECTED) {
    if (wifi_connecting_d.en) {
      serialPrintf("WiFi connected.\n");
      wifi_connecting_d.en = false;
      prs.recvd_qseq = prs.sent_qseq = esp_random();
      prs.recvd_rseq = 0;
      next_udp_d.start(0);
    }
    if (next_udp_d.justFired()) {
      const int missed_responses = (prs.sent_qseq - prs.recvd_qseq) & 0xFFFF;
      if (missed_responses > 0) {
        serialPrintf("UDP timeout #%d\n", missed_responses);
      }
      if (missed_responses >= 5) {
        serialPrintf("Too many UDP timeouts; killing WiFi.\n");
        wifi_connecting_d.start(0);
      } else {
        sendUDP(&prs);
        next_udp_d.start(random(1900, 2100));  // Query again (or timeout) in 2 seconds.
      }
    }
    const ACMode new_mode = receiveUDP(&prs);
    if (new_mode != MODE_INVALID) {
      // Blip to indicate WiFi ok.
      digitalWrite(2, HIGH);
      delay(1);
      digitalWrite(2, LOW);

      if (new_mode != want_mode) {
        want_mode = new_mode;
        print_next_transition();
      }
    }
  } else if (!wifi_connecting_d.en) {
    // WiFi died sometime after a successful connection.
    wifi_connecting_d.start(0);
  }

  if (active_mode_d.elapsed() >= transition_hold_time() &&
      acs.active_mode != want_mode) {
    serialPrintf("Executing transition: %s -> %s\n",
                 MODE_NAMES[acs.active_mode], MODE_NAMES[want_mode]);
    switch (acs.active_mode) {
      case MODE_INVALID:
        resync_d.start(0);
        // fallthrough
      case MODE_OFF:
        // Switching from OFF -> (*) is a good time to stop and sync.
        if (resync_d.justFired()) {
          resyncAndSetActiveMode(want_mode, &acs);
          resync_d.start(RESYNC_MS);
        } else {
          const int off_luma = filteredRead();
          acs.active_mode = acs.memory_mode;
          bool in_menu = false;
          while (acs.active_mode != want_mode) {
            irMode();
            if (!in_menu) {
              in_menu = true;
            } else {
              cycleMode(&acs.active_mode);
            }
          }
          applyActiveMode(in_menu, &acs);
          const int on_luma = filteredRead();
          orderOkOrPanic(MODE_OFF, off_luma, acs.active_mode, on_luma);
        }
        break;
      // HEAT and COOL need to idle before doing anything else.
      case MODE_HEAT:
        acs.heat_temp = irTemp(acs.heat_temp, HEAT_MIN_C);
        acs.active_mode = MODE_HEAT_IDLE;
        compressor_idle_d.start();
        break;      
      case MODE_COOL:
        acs.cool_temp = irTemp(acs.cool_temp, COOL_IDLE_TEMP);
        acs.active_mode = MODE_COOL_IDLE;
        compressor_idle_d.start();
        break;
      case MODE_HEAT_IDLE:
        if (resync_d.justFired()) {
          resyncAndSetActiveMode(want_mode, &acs);
          resync_d.start(RESYNC_MS);
        } else if (want_mode == MODE_HEAT) {
          // Fast idle->heat transition.
          acs.heat_temp = irTemp(acs.heat_temp, HEAT_MAX_C);
          acs.active_mode = MODE_HEAT;
        } else {
          powerOff(&acs);
        }
        break;
      case MODE_COOL_IDLE:
        if (resync_d.justFired()) {
          resyncAndSetActiveMode(want_mode, &acs);
          resync_d.start(RESYNC_MS);
        } else if (want_mode == MODE_COOL) {
          // Fast idle->cool transition.
          acs.cool_temp = irTemp(acs.cool_temp, COOL_RUN_TEMP);
          acs.active_mode = MODE_COOL;
        } else {
          powerOff(&acs);
        }
        break;
      case MODE_FAN:
        powerOff(&acs);
        break;
    }
    if (acs.active_mode == MODE_HEAT || acs.active_mode == MODE_COOL) {
      last_compressor_idle_ms = compressor_idle_d.elapsed();
    }
    active_mode_d.start();
    serialPrintf("Mode is now %s\n", MODE_NAMES[acs.active_mode]);
    print_next_transition();
  }

  delay(1);  // saves 20mA @ 240MHz
}
