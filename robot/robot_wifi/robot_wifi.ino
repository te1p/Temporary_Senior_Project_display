// Robot Duels - Pico W Wi-Fi link + driving
//
// Gets the robot onto Wi-Fi and lets the laptop drive it over UDP, plus a
// status/drive web page.
//
// Network:
//   - no secrets.h  -> robot creates its own network "RobotDuel-<id>"
//   - secrets.h     -> robot joins that router (falls back to its own network)
//
// UDP port 4210, one text command per packet, one reply per packet:
//   hello | info              -> robot id=1 name=robot1 mode=... ip=... battery=...
//   ping <anything>           -> pong <anything>          (round-trip timing)
//   drive <vx> <vy> <turn>    -> ok      each -100..100: forward, slide right, spin clockwise
//   wheels <m1> <m2> <m3> <m4>-> ok      per wheel -100..100 (FL, RL, RR, FR)
//   stop                      -> ok
//   servo <degrees>           -> ok servo N   head angle 30-150, 90 = straight ahead
//   eyes on|off|blink         -> ok eyes ...  LED eyes (they also blink by themselves)
//   led on|off|auto           -> ok led ...
// SAFETY: drive/wheels must be resent at least every FAILSAFE_MS (300 ms) or
// the motors stop by themselves.
//
// Web page: http://<ip>/   (also http://robot<id>.local/)
// Wireless updates: laptop/upload_wifi.sh <robot>   (password: OTA_PASSWORD in secrets.h)
// Board: Raspberry Pi Pico W  (arduino-pico core)
// FQBN:  rp2040:rp2040:rpipicow:flash=2097152_1048576   <- 1 MB storage is needed for Wi-Fi updates

#include <ArduinoOTA.h>
#include <SimpleMDNS.h>  // the mDNS library ArduinoOTA uses; provides robot<id>.local
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "head.h"
#include "motors.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Copy secrets.example.h to secrets.h (same folder) and fill in the Wi-Fi and robot passwords."
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif
#if !defined(AP_PASSWORD) || !defined(OTA_PASSWORD)
#error "secrets.h needs AP_PASSWORD and OTA_PASSWORD - see secrets.example.h"
#endif

WiFiUDP udp;
WebServer web(80);

char hostname[16];  // "robot1"
char apSsid[32];    // "RobotDuel-1"
bool apMode = false;

uint32_t udpCount = 0;
uint32_t lastCmdMs = 0;

bool driving = false;  // motors were commanded and the failsafe is armed
uint32_t lastDriveMs = 0;
uint32_t failsafeStops = 0;
float batteryVolts = 0;

enum LedMode { LED_AUTO, LED_ON, LED_OFF };
LedMode ledMode = LED_AUTO;

// ---------------------------------------------------------------------------
// Wi-Fi
// ---------------------------------------------------------------------------
bool joinRouter() {
  if (strlen(WIFI_SSID) == 0) return false;
  Serial.printf("Joining router \"%s\" ", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < JOIN_TIMEOUT_MS) {
    digitalWrite(LED_BUILTIN, (millis() / 100) % 2);  // fast blink = connecting
    delay(50);
    if ((millis() - start) % 1000 < 50) Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.println("Could not join the router.");
  WiFi.disconnect();
  return false;
}

void startOwnNetwork() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(apSsid, AP_PASSWORD, AP_CHANNEL)) {
    Serial.println("!! Failed to start access point");
  }
  apMode = true;
}

IPAddress myIP() { return apMode ? WiFi.softAPIP() : WiFi.localIP(); }

const char *eyesState() { return !matrixFound ? "not-found" : eyesEnabled ? "on" : "off"; }

String statusLine() {
  char buf[300];
  IPAddress ip = myIP();
  snprintf(buf, sizeof(buf),
           "robot id=%d name=%s mode=%s network=%s ip=%d.%d.%d.%d rssi=%d battery=%.2fV wheels=%d,%d,%d,%d "
           "servo=%d eyes=%s failsafe_stops=%lu up_ms=%lu udp_rx=%lu",
           ROBOT_ID, hostname, apMode ? "ap" : "router", apMode ? apSsid : WIFI_SSID, ip[0], ip[1], ip[2], ip[3],
           apMode ? 0 : (int)WiFi.RSSI(), batteryVolts, motorCmd[0], motorCmd[1], motorCmd[2], motorCmd[3],
           servoAngle, eyesState(), (unsigned long)failsafeStops, (unsigned long)millis(),
           (unsigned long)udpCount);
  return String(buf);
}

// ---------------------------------------------------------------------------
// Driving
// ---------------------------------------------------------------------------
void armFailsafe() {
  driving = true;
  lastDriveMs = millis();
}

void cmdStop() {
  stopMotors();
  driving = false;
}

void checkFailsafe() {
  if (driving && millis() - lastDriveMs > FAILSAFE_MS) {
    stopMotors();
    driving = false;
    failsafeStops++;
    Serial.printf("failsafe: no drive command for %d ms, motors stopped\n", FAILSAFE_MS);
  }
}

// ---------------------------------------------------------------------------
// UDP commands
// ---------------------------------------------------------------------------
void handleUdp() {
  int size = udp.parsePacket();
  if (size <= 0) return;

  char cmd[128];
  int len = udp.read(cmd, sizeof(cmd) - 1);
  if (len < 0) len = 0;
  cmd[len] = 0;
  while (len > 0 && (cmd[len - 1] == '\n' || cmd[len - 1] == '\r' || cmd[len - 1] == ' ')) cmd[--len] = 0;

  udpCount++;
  lastCmdMs = millis();

  String reply;
  int a, b, c, d;
  if (strncmp(cmd, "drive ", 6) == 0) {
    if (sscanf(cmd + 6, "%d %d %d", &a, &b, &c) == 3) {
      drive(a, b, c);
      armFailsafe();
      reply = "ok";
    } else {
      reply = "err usage: drive <vx> <vy> <turn>";
    }
  } else if (strncmp(cmd, "wheels ", 7) == 0) {
    if (sscanf(cmd + 7, "%d %d %d %d", &a, &b, &c, &d) == 4) {
      setWheels(a, b, c, d);
      armFailsafe();
      reply = "ok";
    } else {
      reply = "err usage: wheels <m1> <m2> <m3> <m4>";
    }
  } else if (strcmp(cmd, "stop") == 0) {
    cmdStop();
    reply = "ok";
  } else if (strncmp(cmd, "servo ", 6) == 0) {
    if (sscanf(cmd + 6, "%d", &a) == 1) {
      servoSet(a);
      reply = String("ok servo ") + servoAngle;
    } else {
      reply = "err usage: servo <degrees 30-150>";
    }
  } else if (strncmp(cmd, "eyes ", 5) == 0) {
    const char *arg = cmd + 5;
    if (!matrixFound) reply = "err LED matrix not found on I2C 0x71";
    else if (strcmp(arg, "on") == 0) { eyesSetEnabled(true); reply = "ok eyes on"; }
    else if (strcmp(arg, "off") == 0) { eyesSetEnabled(false); reply = "ok eyes off"; }
    else if (strcmp(arg, "blink") == 0) { eyesBlinkNow(); reply = "ok eyes blink"; }
    else reply = "err usage: eyes on|off|blink";
  } else if (strncmp(cmd, "ping", 4) == 0) {
    reply = String("pong") + (cmd + 4);  // echo whatever followed "ping"
  } else if (strcmp(cmd, "hello") == 0 || strcmp(cmd, "info") == 0) {
    reply = statusLine();
  } else if (strcmp(cmd, "led on") == 0) {
    ledMode = LED_ON;
    reply = "ok led on";
  } else if (strcmp(cmd, "led off") == 0) {
    ledMode = LED_OFF;
    reply = "ok led off";
  } else if (strcmp(cmd, "led auto") == 0) {
    ledMode = LED_AUTO;
    reply = "ok led auto";
  } else {
    reply = String("err unknown command: ") + cmd;
  }

  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write(reply.c_str(), reply.length());
  udp.endPacket();
}

// ---------------------------------------------------------------------------
// Web page
// ---------------------------------------------------------------------------
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>Robot</title>
<style>body{font-family:system-ui,sans-serif;margin:16px;max-width:520px}
pre{background:#f2f2f2;padding:12px;border-radius:8px;white-space:pre-wrap;font-size:13px}
button{font-size:16px;padding:10px 14px;margin:4px 4px 4px 0;border-radius:8px;border:1px solid #888;background:#fff}
.pad{display:grid;grid-template-columns:repeat(3,72px);gap:8px;margin:12px 0;touch-action:none;user-select:none}
.pad button{height:64px;margin:0;font-size:22px}.pad button.on{background:#ffd54a}
.stop{background:#e53935!important;color:#fff}.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
</style></head><body><h2 id="t">Robot</h2>
<p><b>Lift the wheels off the table for the first test.</b> Hold a button (or WASD / arrows, Q/E spin) to drive.
Motors stop by themselves if commands stop for 0.3 s.</p>
<div class="pad" id="pad">
<button data-v="1,-1,0">&#8598;</button><button data-v="1,0,0">&#8593;</button><button data-v="1,1,0">&#8599;</button>
<button data-v="0,-1,0">&#8592;</button><button class="stop" data-v="0,0,0">&#9632;</button><button data-v="0,1,0">&#8594;</button>
<button data-v="-1,-1,0">&#8601;</button><button data-v="-1,0,0">&#8595;</button><button data-v="-1,1,0">&#8600;</button>
<button data-v="0,0,-1">&#10226;</button><span></span><button data-v="0,0,1">&#10227;</button>
</div>
<div class="row">Speed <input id="spd" type="range" min="20" max="100" value="50"><span id="spdv">50</span>%</div>
<div class="row">Head <input id="srv" type="range" min="30" max="150" value="90"><span id="srvv">90</span>&deg;
<button onclick="setServo(90)">Center</button></div>
<div class="row"><button onclick="eyes('blink')">Blink</button><button onclick="eyes('on')">Eyes on</button>
<button onclick="eyes('off')">Eyes off</button></div>
<pre id="s">loading...</pre>
<div class="row"><button onclick="led('on')">LED on</button><button onclick="led('off')">LED off</button>
<button onclick="led('auto')">LED heartbeat</button></div>
<script>
const $=id=>document.getElementById(id);let held=null,timer=null,keys=new Set();
$('spd').oninput=()=>$('spdv').textContent=$('spd').value;
let srvT=0;const srvSend=()=>fetch('/servo?angle='+$('srv').value).catch(()=>{});
$('srv').oninput=()=>{$('srvv').textContent=$('srv').value;if(Date.now()-srvT>60){srvT=Date.now();srvSend();}};
$('srv').onchange=srvSend;
function setServo(a){$('srv').value=a;$('srvv').textContent=a;srvSend();}
async function eyes(m){await fetch('/eyes?mode='+m).catch(()=>{});refresh();}
function go(v){const s=+$('spd').value;fetch(`/drive?vx=${v[0]*s}&vy=${v[1]*s}&turn=${v[2]*s}`).catch(()=>{});}
function start(v){held=v;go(v);clearInterval(timer);timer=setInterval(()=>held&&go(held),100);}
function end(){held=null;clearInterval(timer);fetch('/stop').catch(()=>{});}
document.querySelectorAll('#pad button').forEach(b=>{const v=b.dataset.v.split(',').map(Number);
 b.onpointerdown=e=>{e.preventDefault();b.classList.add('on');(v[0]||v[1]||v[2])?start(v):end();};
 b.onpointerup=b.onpointerleave=b.onpointercancel=()=>{if(b.classList.contains('on')){b.classList.remove('on');end();}};});
const KM={w:[1,0,0],arrowup:[1,0,0],s:[-1,0,0],arrowdown:[-1,0,0],a:[0,-1,0],arrowleft:[0,-1,0],
 d:[0,1,0],arrowright:[0,1,0],q:[0,0,-1],e:[0,0,1]};
function fromKeys(){let v=[0,0,0];keys.forEach(k=>KM[k].forEach((x,i)=>v[i]+=x));v=v.map(x=>Math.max(-1,Math.min(1,x)));
 (v[0]||v[1]||v[2])?start(v):end();}
onkeydown=e=>{const k=e.key.toLowerCase();if(k===' '){keys.clear();end();return;}if(!KM[k]||keys.has(k))return;
 e.preventDefault();keys.add(k);fromKeys();};
onkeyup=e=>{const k=e.key.toLowerCase();if(keys.delete(k))fromKeys();};
onblur=()=>{keys.clear();end();};
async function refresh(){try{const j=await(await fetch('/status')).json();
$('t').textContent='Robot '+j.id+' ('+j.name+')';
$('s').textContent=Object.entries(j).map(([k,v])=>k+': '+v).join('\n');}catch(e){}}
async function led(m){await fetch('/led?mode='+m);refresh();}
refresh();setInterval(refresh,1000);
</script></body></html>)HTML";

void handleStatus() {
  IPAddress ip = myIP();
  char json[600];
  snprintf(json, sizeof(json),
           "{\"id\":%d,\"name\":\"%s\",\"mode\":\"%s\",\"network\":\"%s\",\"ip\":\"%d.%d.%d.%d\","
           "\"rssi_dbm\":%d,\"battery_v\":%.2f,\"wheels_FL_RL_RR_FR\":\"%d, %d, %d, %d\",\"driving\":%s,"
           "\"servo_deg\":%d,\"eyes\":\"%s\","
           "\"failsafe_stops\":%lu,\"uptime_s\":%lu,\"udp_packets\":%lu,\"last_cmd_ms_ago\":%ld,\"led\":\"%s\"}",
           ROBOT_ID, hostname, apMode ? "own network" : "router", apMode ? apSsid : WIFI_SSID, ip[0], ip[1],
           ip[2], ip[3], apMode ? 0 : (int)WiFi.RSSI(), batteryVolts, motorCmd[0], motorCmd[1], motorCmd[2],
           motorCmd[3], driving ? "true" : "false", servoAngle, eyesState(), (unsigned long)failsafeStops,
           (unsigned long)(millis() / 1000),
           (unsigned long)udpCount, lastCmdMs ? (long)(millis() - lastCmdMs) : -1L,
           ledMode == LED_AUTO ? "heartbeat" : ledMode == LED_ON ? "on" : "off");
  web.send(200, "application/json", json);
}

void handleLed() {
  String m = web.arg("mode");
  ledMode = m == "on" ? LED_ON : m == "off" ? LED_OFF : LED_AUTO;
  web.send(200, "text/plain", "ok");
}

void handleDrive() {
  drive(web.arg("vx").toInt(), web.arg("vy").toInt(), web.arg("turn").toInt());
  armFailsafe();
  lastCmdMs = millis();
  web.send(200, "text/plain", "ok");
}

void handleStop() {
  cmdStop();
  web.send(200, "text/plain", "ok");
}

void handleServo() {
  servoSet(web.arg("angle").toInt());
  web.send(200, "text/plain", String(servoAngle));
}

void handleEyes() {
  String m = web.arg("mode");
  if (m == "blink") eyesBlinkNow();
  else if (matrixFound) eyesSetEnabled(m != "off");
  web.send(200, "text/plain", eyesState());
}

// ---------------------------------------------------------------------------
void updateLed() {
  bool on;
  if (ledMode == LED_ON) on = true;
  else if (ledMode == LED_OFF) on = false;
  else if (driving) on = (millis() % 200) < 100;  // fast blink while driving
  else on = (millis() % 1000) < (apMode ? 500 : 80);  // own network: slow blink, router: short heartbeat
  digitalWrite(LED_BUILTIN, on);
}

void setup() {
  motorsBegin();  // first thing: make sure the motors are off
  servoBegin();   // head straight ahead
  eyesBegin();    // eyes open
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);  // give the USB serial monitor a moment

  snprintf(hostname, sizeof(hostname), "robot%d", ROBOT_ID);
  snprintf(apSsid, sizeof(apSsid), "RobotDuel-%d", ROBOT_ID);
  Serial.printf("\n=== Robot Duels robot %d ===\n", ROBOT_ID);
  Serial.printf("LED eyes: %s\n", matrixFound ? "found at I2C 0x71" : "NOT FOUND (check the 4-wire cable to the matrix)");

  if (!joinRouter()) startOwnNetwork();
  WiFi.noLowPowerMode();  // Wi-Fi power saving adds tens of ms of latency

  udp.begin(UDP_PORT);
  web.on("/", []() { web.send_P(200, "text/html", PAGE); });
  web.on("/status", handleStatus);
  web.on("/led", handleLed);
  web.on("/drive", handleDrive);
  web.on("/stop", handleStop);
  web.on("/servo", handleServo);
  web.on("/eyes", handleEyes);
  web.begin();

  // Wireless updates. ArduinoOTA also starts mDNS (robot<id>.local).
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    cmdStop();  // never drive while the firmware is being replaced
    Serial.println("Wi-Fi update starting - motors stopped");
    if (matrixFound) matrixShow(EYE_SHUT, EYE_SHUT);
  });
  ArduinoOTA.onEnd([]() { Serial.println("Wi-Fi update received - restarting"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("Wi-Fi update failed (error %d)\n", (int)e); });
  ArduinoOTA.begin();
  MDNS.addService("robotduel", "udp", UDP_PORT);

  batteryVolts = readBatteryVolts();
  Serial.println(statusLine());
  if (apMode)
    Serial.printf("Join Wi-Fi \"%s\" (password: AP_PASSWORD in secrets.h), then open http://%s/\n", apSsid,
                  myIP().toString().c_str());
  else
    Serial.printf("Open http://%s/ or http://%s.local/\n", myIP().toString().c_str(), hostname);
}

void loop() {
  handleUdp();
  web.handleClient();
  checkFailsafe();
  ArduinoOTA.handle();  // also keeps mDNS running
  updateLed();
  eyesUpdate();

  static uint32_t lastBattery = 0;
  if (millis() - lastBattery > 500) {
    lastBattery = millis();
    batteryVolts = readBatteryVolts();
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    lastPrint = millis();
    Serial.println(statusLine());
  }
}
