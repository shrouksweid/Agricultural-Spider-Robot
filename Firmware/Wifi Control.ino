/*
 * ============================================================================
 *  Agricultural Spider Robot - ESP32 MQTT Communication Subsystem
 * ============================================================================
 *
 *  File:           spider_robot.ino
 *  Target:         ESP32 Dev Module (Arduino Core)
 *  Environment:    Arduino IDE
 *  Libraries:      WiFi.h, PubSubClient.h (by Nick O'Leary), ArduinoJson.h (v6+)
 *
 *  Description:
 *      Production-ready MQTT communication subsystem for an agricultural
 *      quadruped/hexapod spider robot. Handles WiFi + MQTT connectivity,
 *      command dispatch (movement / behavior / tool), GPIO actuation of the
 *      sprayer pump and cutter blade MOSFETs, and periodic telemetry
 *      publication. Designed as a non-blocking, modular architecture that
 *      can be extended with locomotion kinematics without restructuring.
 *
 *  Network:
 *      Broker  : broker.hivemq.com   (public HiveMQ broker)
 *      Port    : 1883                (plain MQTT, no TLS)
 *
 *  Topic Tree:
 *      SUBSCRIBE   spider/move       (forward | backward | left | right | stop)
 *      SUBSCRIBE   spider/behavior   (stand | sleep | highfive | happy | wave | dance)
 *      SUBSCRIBE   spider/tool       (sprayer_on | sprayer_off | cutter_on | cutter_off)
 *      PUBLISH     spider/status     (JSON telemetry, 1 Hz)
 *
 *  GPIO Map:
 *      GPIO16  -> Sprayer Pump MOSFET gate   (active HIGH)
 *      GPIO17  -> Cutter Blade MOSFET gate   (active HIGH)
 *
 *  Author: Graduation Project - Agricultural Spider Robot
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* ============================================================================
 *  1. CONFIGURATION
 * ============================================================================ */

/* ---- WiFi Credentials ---- */
static const char* WIFI_SSID     = "Vodafone_VDSL";
static const char* WIFI_PASSWORD = "UHs2Q9D6kTchPRcb";

/* ---- MQTT Broker ---- */
static const char* MQTT_BROKER   = "broker.hivemq.com";
static const uint16_t MQTT_PORT  = 1883;
static const char* MQTT_CLIENT_ID = "spider_robot_esp32";

/* ---- MQTT Topics ---- */
static const char* TOPIC_MOVE     = "spider/move";
static const char* TOPIC_BEHAVIOR = "spider/behavior";
static const char* TOPIC_TOOL     = "spider/tool";
static const char* TOPIC_STATUS   = "spider/status";

/* ---- GPIO Pin Assignments ---- */
static const uint8_t PIN_SPRAYER  = 16;   // Pump MOSFET gate
static const uint8_t PIN_CUTTER   = 17;   // Blade MOSFET gate

/* ---- Timing ---- */
static const uint32_t TELEMETRY_INTERVAL_MS = 1000UL;   // 1 Hz publish
static const uint32_t MQTT_RETRY_INTERVAL_MS = 2000UL;  // Backoff on disconnect
static const uint32_t WIFI_RETRY_INTERVAL_MS = 2000UL;  // Backoff on disconnect
static const uint16_t MQTT_KEEPALIVE_SEC    = 15;
static const uint32_t MQTT_SOCKET_TIMEOUT_MS = 5000;

/* ---- Telemetry JSON buffer size ---- */
static const size_t  TELEMETRY_JSON_SIZE = 256;

/* ============================================================================
 *  2. GLOBAL STATE
 * ============================================================================ */

WiFiClient    netClient;
PubSubClient  mqttClient(netClient);

/* Runtime state machine ----------------------------------------------- */
struct RobotState {
  String    currentBehavior;   // stand | sleep | highfive | happy | wave | dance
  bool      sprayerOn;
  bool      cutterOn;
  uint8_t   batteryPercent;    // Placeholder until ADC battery monitor wired
  bool      wifiConnected;
  bool      mqttConnected;
  uint32_t  bootTimeMs;
};

static RobotState robotState = {
  "stand", false, false, 85, false, false, 0
};

/* Timing trackers (non-blocking scheduler) ---------------------------- */
static uint32_t lastTelemetryMs  = 0;
static uint32_t lastMqttRetryMs  = 0;
static uint32_t lastWifiRetryMs  = 0;

/* ============================================================================
 *  3. SERIAL LOGGER (tagged, single source of truth)
 * ============================================================================ */

static void logTag(const char* tag, const char* msg) {
  Serial.printf("[%s] %s\n", tag, msg);
}
static void logTagF(const char* tag, const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.printf("[%s] %s\n", tag, buf);
}

/* ============================================================================
 *  4. WiFi MANAGER
 * ============================================================================ */

static void wifiBegin() {
  logTag("WIFI", "Connecting to SSID...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);            // SDK auto-reconnect
  WiFi.persistent(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

/* Non-blocking WiFi state machine. Returns true once IP is acquired. */
static bool wifiMaintain() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!robotState.wifiConnected) {
      robotState.wifiConnected = true;
      logTagF("WIFI", "Connected. IP=%s, RSSI=%d dBm",
              WiFi.localIP().toString().c_str(),
              WiFi.RSSI());
    }
    return true;
  }

  /* Disconnected path */
  if (robotState.wifiConnected) {
    robotState.wifiConnected = false;
    logTag("WIFI", "Connection lost. Auto-reconnecting...");
  }

  uint32_t now = millis();
  if (now - lastWifiRetryMs >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetryMs = now;
    logTag("WIFI", "Retrying connection...");
  }
  return false;
}

/* ============================================================================
 *  5. GPIO DRIVERS - Sprayer & Cutter
 * ============================================================================ */

static void driversInit() {
  pinMode(PIN_SPRAYER, OUTPUT);
  pinMode(PIN_CUTTER,  OUTPUT);
  digitalWrite(PIN_SPRAYER, LOW);
  digitalWrite(PIN_CUTTER,  LOW);
  logTagF("GPIO", "Sprayer=GPIO%d, Cutter=GPIO%d (OFF)", PIN_SPRAYER, PIN_CUTTER);
}

static void setSprayer(bool on) {
  digitalWrite(PIN_SPRAYER, on ? HIGH : LOW);
  robotState.sprayerOn = on;
  logTagF("GPIO", "Sprayer MOSFET -> %s", on ? "ON" : "OFF");
}

static void setCutter(bool on) {
  digitalWrite(PIN_CUTTER, on ? HIGH : LOW);
  robotState.cutterOn = on;
  logTagF("GPIO", "Cutter MOSFET -> %s", on ? "ON" : "OFF");
}

/* ============================================================================
 *  6. COMMAND HANDLERS  (modular, swappable for real locomotion code)
 * ============================================================================ *
 *
 *  Each handler is a standalone function. To integrate real locomotion
 *  kinematics later, replace the body of handleMovement() with calls to
 *  your gait controller (e.g. spiderGait.forward(velocity)), keeping the
 *  same signature so the dispatcher does not need to change.
 */

/* ---- 6.1 Movement ---------------------------------------------------- */

static void handleMovement(const String& cmd) {
  /*
   * Future integration point:
   *   forward  -> locomotion.setVelocity(+V, 0)
   *   backward -> locomotion.setVelocity(-V, 0)
   *   left     -> locomotion.setYawRate(-W)
   *   right    -> locomotion.setYawRate(+W)
   *   stop     -> locomotion.halt()
   */
  if (cmd == "forward" || cmd == "backward" ||
      cmd == "left"    || cmd == "right"    ||
      cmd == "stop") {
    logTag("MOVE", cmd.c_str());
    /* Placeholder: real locomotion call goes here. */
  } else {
    logTagF("MOVE", "Unknown command: %s", cmd.c_str());
  }
}

/* ---- 6.2 Behavior ---------------------------------------------------- */

static void handleBehavior(const String& cmd) {
  if (cmd == "stand" || cmd == "sleep" ||
      cmd == "highfive" || cmd == "happy" ||
      cmd == "wave" || cmd == "dance") {
    robotState.currentBehavior = cmd;
    logTag("BEHAVIOR", cmd.c_str());
    /* Placeholder: trigger posture/gesture routine on the locomotion layer. */
  } else {
    logTagF("BEHAVIOR", "Unknown command: %s", cmd.c_str());
  }
}

/* ---- 6.3 Tools ------------------------------------------------------- */

static void handleTool(const String& cmd) {
  if      (cmd == "sprayer_on")  { setSprayer(true);  logTag("TOOL", "sprayer_on");  }
  else if (cmd == "sprayer_off") { setSprayer(false); logTag("TOOL", "sprayer_off"); }
  else if (cmd == "cutter_on")   { setCutter(true);   logTag("TOOL", "cutter_on");   }
  else if (cmd == "cutter_off")  { setCutter(false);  logTag("TOOL", "cutter_off");  }
  else {
    logTagF("TOOL", "Unknown command: %s", cmd.c_str());
  }
}

/* ============================================================================
 *  7. MQTT CALLBACK - Dispatcher
 * ============================================================================ */

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  /* Copy payload into a null-terminated buffer (safe for String operations) */
  char buf[64];
  if (length >= sizeof(buf)) length = sizeof(buf) - 1;
  memcpy(buf, payload, length);
  buf[length] = '\0';

  /* Trim trailing whitespace/newlines */
  String cmd = String(buf);
  cmd.trim();
  String tpc = String(topic);

  logTagF("MQTT", "RX %s : %s", tpc.c_str(), cmd.c_str());

  /* Dispatch by topic */
  if      (tpc == TOPIC_MOVE)     handleMovement(cmd);
  else if (tpc == TOPIC_BEHAVIOR) handleBehavior(cmd);
  else if (tpc == TOPIC_TOOL)     handleTool(cmd);
  else                            logTagF("MQTT", "Unhandled topic: %s", tpc.c_str());
}

/* ============================================================================
 *  8. MQTT MANAGER
 * ============================================================================ */

static void mqttBegin() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE_SEC);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_MS / 1000);
  mqttClient.setBufferSize(512);    // Enough for telemetry JSON + overhead
  mqttClient.setCallback(mqttCallback);
}

static bool mqttConnect() {
  logTagF("MQTT", "Connecting to %s:%u ...", MQTT_BROKER, MQTT_PORT);

  bool ok = mqttClient.connect(MQTT_CLIENT_ID);
  if (!ok) {
    logTagF("MQTT", "Connect failed, state=%d", mqttClient.state());
    return false;
  }

  logTag("MQTT", "Connected");

  /* Subscribe to all command topics */
  bool s1 = mqttClient.subscribe(TOPIC_MOVE);
  bool s2 = mqttClient.subscribe(TOPIC_BEHAVIOR);
  bool s3 = mqttClient.subscribe(TOPIC_TOOL);

  logTagF("MQTT", "Subscribed  move=%d  behavior=%d  tool=%d",
          s1 ? 1 : 0, s2 ? 1 : 0, s3 ? 1 : 0);
  return true;
}

static void mqttMaintain() {
  /* PubSubClient.loop() must be called frequently to process network I/O */
  mqttClient.loop();

  if (mqttClient.connected()) {
    if (!robotState.mqttConnected) {
      robotState.mqttConnected = true;
    }
    return;
  }

  /* Disconnected - reconnect with backoff */
  if (robotState.mqttConnected) {
    robotState.mqttConnected = false;
    logTag("MQTT", "Disconnected. Will retry...");
  }

  uint32_t now = millis();
  if (now - lastMqttRetryMs >= MQTT_RETRY_INTERVAL_MS) {
    lastMqttRetryMs = now;
    if (robotState.wifiConnected) {
      mqttConnect();
    } else {
      logTag("MQTT", "Skipping reconnect - WiFi down");
    }
  }
}

/* ============================================================================
 *  9. TELEMETRY PUBLISHER
 * ============================================================================ */

static void publishTelemetry() {
  StaticJsonDocument<TELEMETRY_JSON_SIZE> doc;
  doc["battery"] = robotState.batteryPercent;
  doc["wifi"]    = robotState.wifiConnected ? "connected" : "disconnected";
  doc["state"]   = robotState.currentBehavior;
  doc["sprayer"] = robotState.sprayerOn ? "on" : "off";
  doc["cutter"]  = robotState.cutterOn  ? "on" : "off";
  doc["uptime"]  = (uint32_t)(millis() - robotState.bootTimeMs) / 1000UL;

  char payload[TELEMETRY_JSON_SIZE];
  size_t written = serializeJson(doc, payload, sizeof(payload));
  if (written == 0) {
    logTag("STATUS", "JSON serialization failed");
    return;
  }

  bool ok = mqttClient.publish(TOPIC_STATUS, payload, false);
  if (ok) {
    logTag("STATUS", "Published");
  } else {
    logTagF("STATUS", "Publish failed, mqtt_state=%d", mqttClient.state());
  }
}

/* ============================================================================
 *  10. ARDUINO LIFECYCLE
 * ============================================================================ */

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println(F("==========================================================="));
  Serial.println(F("  Agricultural Spider Robot - ESP32 MQTT Subsystem v1.0   "));
  Serial.println(F("==========================================================="));

  robotState.bootTimeMs = millis();

  driversInit();
  wifiBegin();
  mqttBegin();

  logTag("BOOT", "Initialization complete");
}

void loop() {
  /* 1. Maintain WiFi (non-blocking) */
  wifiMaintain();

  /* 2. Maintain MQTT (non-blocking) */
  mqttMaintain();

  /* 3. Periodic telemetry publish (1 Hz, non-blocking) */
  uint32_t now = millis();
  if (robotState.mqttConnected &&
      (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)) {
    lastTelemetryMs = now;
    publishTelemetry();
  }

  /* 4. Yield to lower-priority tasks (WDT-friendly) */
  yield();
}
