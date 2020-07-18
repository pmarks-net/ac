#include <WiFi.h>
#include <cstdio>
#include <map>
#include <deque>

const char WIFI_SSID[] = "redacted";
const char WIFI_PASS[] = "redacted";
const int SERVER_PORT = 4533;

#define ARRAY_SIZE(array) ((sizeof(array))/(sizeof(array[0])))

template<typename ... Args>
void serialPrintf(const char* format, Args ... args) {
  char buf[256] = "";
  snprintf(buf, sizeof(buf), format, args ...);
  Serial.print(buf);
}

// Assuming the USB port is "down":
// - Pin 18: right connector, orange, labeled "O" on the thermostat.
//   "heat pump changeover valve" energized (active low) for cooling mode,
//   deenergized for heating mode.
// - Pin 19: right connector, yellow, labeled "Y1" on the thermostat.
//   "compressor relay stage 1" energized (active low) to activate
//   the heat pump compressor, otherwise off.
const int OPTO_PINS[] = {18, 19};
int opto_count = 0;
int opto_lo[ARRAY_SIZE(OPTO_PINS)] = {0};
unsigned int opto_sample_millis = 0;
std::deque<int> opto_queue;

void OptoSample() {
  for (int i = 0; i < ARRAY_SIZE(OPTO_PINS); i++) {
    if (!digitalRead(OPTO_PINS[i])) {
      opto_lo[i]++;
    }
  }
  opto_count++;
}

int ReduceOptoSamples() {
  int out_bits = 0;
  for (int i = 0; i < ARRAY_SIZE(OPTO_PINS); i++) {
    // Set bit 'i' if >75% of samples are low (AC waveform present.)
    if (opto_lo[i]*4 > opto_count*3) {
      out_bits |= (1 << i); 
    }
    opto_lo[i] = 0;
  }
  opto_count = 0;
  return out_bits;
}

// Returns true if the queue contains 10 identical samples.
// Each sample is based on 25ms of AC waveform, so 250ms total.
bool RotateOptoQueue(int sample) {
  static const int MAX_SIZE = 10;
  while (opto_queue.size() >= MAX_SIZE) {
    opto_queue.pop_front();
  }
  opto_queue.push_back(sample);
  int count_same = 0;
  for (int v : opto_queue) {
    if (v == opto_queue[0]) {
      count_same++;
    }
  }
  return count_same == MAX_SIZE;
}

bool DoOptoStuff() {
  const unsigned long now = millis();
  if (opto_sample_millis != now) {
    OptoSample();
    opto_sample_millis = now;
    if (opto_count >= 25 /* milliseconds */) {
      return RotateOptoQueue(ReduceOptoSamples());
    }
  }
  return false;  // no new value
}

struct IP4Port {
  IP4Port(IPAddress ip0, uint16_t port0) {
    ip = ip0;
    port = port0;
  }
  IPAddress ip;
  uint16_t port;
};

struct IP4PortCmp {
  bool operator()(const IP4Port& a, const IP4Port& b) {
    // Endian doesn't matter, as long as the order is consistent.
    if (a.ip != b.ip) {
      return a.ip < b.ip;
    }
    return a.port < b.port;
  }
};

struct Seqs {
  uint16_t qseq;
  int rseq = -1;
};

std::map<IP4Port, Seqs, IP4PortCmp> clients;
unsigned long cleanup_millis = millis();

WiFiUDP udp;

String send_mode = "OFF";

void sendUDP(const struct IP4Port& dest, const struct Seqs& cs) {
  char buf[32];
  const int buf_len = sprintf(buf, "R %04x %x %s",
                              cs.qseq, cs.rseq, send_mode.c_str());
  udp.beginPacket(dest.ip, dest.port);
  udp.write(reinterpret_cast<const uint8_t*>(buf), buf_len);
  udp.endPacket();
}

void pokeAllClients(bool actually_send) {
  for (auto it = clients.begin(); it != clients.end();) {
    const IP4Port& client = it->first;
    Seqs* cs = &it->second;
    if (++cs->rseq < 5) {
      if (actually_send) {
        sendUDP(client, *cs);
      }
      ++it;
    } else {      
      serialPrintf("Dropping client: %s %d\n",
                   client.ip.toString().c_str(), client.port);
      it = clients.erase(it);
    }
  }
}

void setup() {
  Serial.begin(115200);
  serialPrintf("Hello from relay_server\n");

  setCpuFrequencyMhz(80);  // saves 35mA
  serialPrintf("CPU: %d MHz\n", getCpuFrequencyMhz());
  
  pinMode(2, OUTPUT);

  for (int pin : OPTO_PINS) {
    pinMode(pin, INPUT_PULLUP);
  }

  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  udp.begin(SERVER_PORT);
}

void loop() {
  bool blink = false;
  
  if (DoOptoStuff()) {
    String new_mode;
    switch (opto_queue[0]) {
      case 0x2:  // Y1 active, O inactive
        new_mode = "HEAT";
        break;
      case 0x3:  // Y1 active, O active
        new_mode = "COOL";
        break;
      default:   // Y1 inactive
        new_mode = "OFF";
        break;
    }
    if (new_mode != send_mode) {
      serialPrintf("Changing from %s->%s\n",
                   send_mode.c_str(), new_mode.c_str());
      send_mode = new_mode;
      pokeAllClients(true);
    }
  }

  while (udp.parsePacket() > 0) {
    char buf[32];
    const int buf_len = udp.read(buf, ARRAY_SIZE(buf)-1);
    if (buf_len < 0) continue;
    buf[buf_len] = '\0';
    char buf_type;
    uint16_t buf_qseq;
    char buf_ignore;
    if (std::sscanf(buf, "%c %04" SCNx16 "%c", &buf_type, &buf_qseq, &buf_ignore) != 2) {
      serialPrintf("packet malformed\n");
      continue;
    }
    if (buf_type != 'Q') {
      serialPrintf("packet with type '%c'\n", buf_type);
      continue;
    }
    const IP4Port client(udp.remoteIP(), udp.remotePort());
    Seqs* cs = &clients[client];
    if (cs->rseq < 0) {
      serialPrintf("New client: %s %d\n",
                   client.ip.toString().c_str(), client.port);
    } else if (((cs->qseq - buf_qseq) & 0xFFFF) < 10) {
      serialPrintf("packet out of order: %04x->%04x\n", cs->qseq, buf_qseq);
      continue;
    }
    cs->qseq = buf_qseq;
    cs->rseq = 0;
    sendUDP(client, *cs);
    blink = true;
  }

  const unsigned long now = millis();
  if (now - cleanup_millis >= 10000) {
    pokeAllClients(false);
    cleanup_millis = now;
  }

  if (blink) digitalWrite(2, HIGH);
  delay(1);  // saves 5mA @ 80MHz
  if (blink) digitalWrite(2, LOW);
}
