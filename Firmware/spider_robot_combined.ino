/* ============================================================================
 * Agricultural Spider Robot - ESP32 Combined Master Code
 * ============================================================================
 *
 * File: spider_robot_combined.ino
 * Target: ESP32 Dev Module (Arduino Core)
 * Environment: Arduino IDE
 * Libraries: WiFi.h, PubSubClient.h (by Nick O'Leary), ArduinoJson.h (v6+)
 *             Wire.h, Adafruit_GFX.h, Adafruit_SSD1306.h
 *
 * Description:
 *   Combined master code for the Agricultural Spider Robot.
 *   Integrates three subsystems:
 *     1. WiFi + MQTT communication (movement / behavior / tool commands)
 *     2. OLED SSD1306 facial expressions & servo pose controller
 *     3. DC cutter motor (PWM via LEDC) and water pump (GPIO)
 *
 *   A fourth subsystem (Inverse Kinematics + Bezier curves) will be
 *   integrated in a future merge — setServoAngle() is intentionally left
 *   as a stub to receive that code without restructuring.
 *
 * Network:
 *   Broker : broker.hivemq.com (public HiveMQ broker)
 *   Port   : 1883 (plain MQTT, no TLS)
 *
 * Topic Tree:
 *   SUBSCRIBE  spider/move      (forward | backward | left | right | stop)
 *   SUBSCRIBE  spider/behavior  (stand | sleep | highfive | happy | wave | dance)
 *   SUBSCRIBE  spider/tool      (sprayer_on | sprayer_off | cutter_on | cutter_off |
 *                                cutter_high | cutter_medium)
 *   PUBLISH    spider/status    (JSON telemetry, 1 Hz)
 *
 * GPIO Map (ESP32 — resolved, no conflicts):
 *   GPIO21 -> I2C SDA  (OLED)
 *   GPIO22 -> I2C SCL  (OLED)
 *   GPIO25 -> Water Pump  (active HIGH, simple GPIO)
 *   GPIO26 -> Cutter Blade Motor  (PWM via LEDC channel 0)
 *
 * Author: Graduation Project - Agricultural Spider Robot
 * ============================================================================ */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ============================================================================
 * 1. CONFIGURATION
 * ============================================================================ */

/* ---- WiFi Credentials ---- */
static const char* WIFI_SSID       = "Vodafone_VDSL";
static const char* WIFI_PASSWORD   = "UHs2Q9D6kTchPRcb";

/* ---- MQTT Broker ---- */
static const char* MQTT_BROKER     = "broker.hivemq.com";
static const uint16_t MQTT_PORT    = 1883;
static const char* MQTT_CLIENT_ID  = "spider_robot_esp32";

/* ---- MQTT Topics ---- */
static const char* TOPIC_MOVE      = "spider/move";
static const char* TOPIC_BEHAVIOR  = "spider/behavior";
static const char* TOPIC_TOOL      = "spider/tool";
static const char* TOPIC_STATUS    = "spider/status";

/* ---- GPIO Pin Assignments ---- */
// Tools
static const uint8_t PUMP_PIN      = 25;   // Water pump  (active HIGH, simple GPIO)
static const uint8_t Cutter_PIN    = 26;   // Cutter blade motor (PWM via LEDC ch 0)

/* ---- Timing ---- */
static const uint32_t TELEMETRY_INTERVAL_MS  = 1000UL;
static const uint32_t MQTT_RETRY_INTERVAL_MS = 2000UL;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 2000UL;
static const uint16_t MQTT_KEEPALIVE_SEC     = 15;
static const uint32_t MQTT_SOCKET_TIMEOUT_MS = 5000;

/* ---- Telemetry JSON buffer size ---- */
static const size_t TELEMETRY_JSON_SIZE = 256;

/* ============================================================================
 * 2. OLED DISPLAY SETUP
 * ============================================================================ */

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1       // Shared with Arduino reset
#define SCREEN_ADDRESS  0x3C     // Default I2C address for SSD1306

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum FaceAnimMode : uint8_t {
    FACE_ANIM_LOOP      = 0,   // Continuous looping (natural blink)
    FACE_ANIM_ONCE      = 1,   // Play animation once only
    FACE_ANIM_BOOMERANG = 2    // Pendulum back-and-forth
};

String       currentFace      = "stand";
FaceAnimMode currentFaceMode  = FACE_ANIM_LOOP;
unsigned long lastAnimationUpdate = 0;
int          animationFrame   = 0;

/* ============================================================================
 * 3. SERVO MAPPING
 * ============================================================================
 * 4-legged robot, 2 servos per leg (hip + knee).
 * setServoAngle() is a stub — Inverse Kinematics code will fill it in the
 * next merge without requiring any structural changes here.
 * ============================================================================ */

enum ServoName : uint8_t {
    R1 = 0,   // Right Front Hip
    R2 = 1,   // Right Back  Hip
    L1 = 2,   // Left  Front Hip
    L2 = 3,   // Left  Back  Hip
    R4 = 4,   // Right Front Knee
    R3 = 5,   // Right Back  Knee
    L3 = 6,   // Left  Front Knee
    L4 = 7    // Left  Back  Knee
};

int  frameDelay    = 150;   // Delay between motion frames (ms)
int  walkCycles    = 4;     // Steps per walk command

/* ============================================================================
 * 4. GLOBAL STATE
 * ============================================================================ */

WiFiClient    netClient;
PubSubClient  mqttClient(netClient);

struct RobotState {
    String   currentBehavior;   // stand | sleep | highfive | happy | wave | dance
    bool     sprayerOn;
    bool     cutterOn;
    uint8_t  batteryPercent;    // Placeholder until ADC battery monitor is wired
    bool     wifiConnected;
    bool     mqttConnected;
    uint32_t bootTimeMs;
};

static RobotState robotState = { "stand", false, false, 85, false, false, 0 };

// currentCommand is shared between MQTT dispatcher and pose routines
String currentCommand = "";

/* Timing trackers (non-blocking scheduler) */
static uint32_t lastTelemetryMs  = 0;
static uint32_t lastMqttRetryMs  = 0;
static uint32_t lastWifiRetryMs  = 0;

/* ============================================================================
 * 5. SERIAL LOGGER
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
 * 6. WIFI MANAGER
 * ============================================================================ */

static void wifiBegin() {
    logTag("WIFI", "Connecting to SSID...");
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

static bool wifiMaintain() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!robotState.wifiConnected) {
            robotState.wifiConnected = true;
            logTagF("WIFI", "Connected. IP=%s, RSSI=%d dBm",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
        }
        return true;
    }
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
 * 7. CUTTER & PUMP DRIVERS
 * ============================================================================ */

static void driversInit() {
    // Cutter blade motor — PWM via ESP32 LEDC (channel 0, 5 kHz, 8-bit)
    ledcAttach(Cutter_PIN, 5000, 8);
    ledcWrite(Cutter_PIN, 0);   // Start OFF

    // Water pump — simple GPIO
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    logTagF("GPIO", "Pump=GPIO%d (OFF), Cutter=GPIO%d PWM (OFF)", PUMP_PIN, Cutter_PIN);
}

// Cutter motor speed levels
void CutterHigh() {
    ledcWrite(Cutter_PIN, 255);
    robotState.cutterOn = true;
    logTag("CUTTER", "HIGH speed");
}

void CutterMedium() {
    ledcWrite(Cutter_PIN, 125);
    robotState.cutterOn = true;
    logTag("CUTTER", "MEDIUM speed");
}

void CutterOff() {
    ledcWrite(Cutter_PIN, 0);
    robotState.cutterOn = false;
    logTag("CUTTER", "OFF");
}

// Water pump
void pumpOn() {
    digitalWrite(PUMP_PIN, HIGH);
    robotState.sprayerOn = true;
    logTag("PUMP", "ON");
}

void pumpOff() {
    digitalWrite(PUMP_PIN, LOW);
    robotState.sprayerOn = false;
    logTag("PUMP", "OFF");
}

// Spray for a specific duration (blocking — use only from command context)
void spray(unsigned long duration) {
    pumpOn();
    delay(duration);
    pumpOff();
}

/* ============================================================================
 * 8. OLED FACE RENDERING
 * ============================================================================ */

void drawEyes(int eyeHeight, int pupYOffset = 0) {
    display.clearDisplay();
    display.fillRoundRect(20, 32 - (eyeHeight / 2) + pupYOffset, 30, eyeHeight, 8, SSD1306_WHITE);
    display.fillRoundRect(78, 32 - (eyeHeight / 2) + pupYOffset, 30, eyeHeight, 8, SSD1306_WHITE);
    display.display();
}

void drawAngryEyes() {
    display.clearDisplay();
    display.fillRoundRect(20, 20, 30, 25, 4, SSD1306_WHITE);
    display.fillRoundRect(78, 20, 30, 25, 4, SSD1306_WHITE);
    // Black triangles over the upper portion to make eyes look sharp/angry
    display.fillTriangle(20, 20, 50, 20, 20, 30, SSD1306_BLACK);
    display.fillTriangle(78, 20, 108, 20, 108, 30, SSD1306_BLACK);
    display.display();
}

void updateFaceAnimation() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastAnimationUpdate < 100) return;
    lastAnimationUpdate = currentMillis;

    if (currentFace == "stand" || currentFace == "walk" ||
        currentFace == "wave"  || currentFace == "dance") {
        // Natural blink cycle
        if      (animationFrame == 0) drawEyes(25);
        else if (animationFrame == 1) drawEyes(15);
        else if (animationFrame == 2) drawEyes(5);
        else if (animationFrame == 3) drawEyes(15);
        animationFrame = (animationFrame + 1) % 4;
    } else if (currentFace == "angry" || currentFace == "pushup") {
        drawAngryEyes();
    } else if (currentFace == "rest" || currentFace == "dead") {
        drawEyes(4);   // Sleepy / closed eyes
    } else {
        drawEyes(25);  // Default open eyes
    }
}

// Non-blocking delay that keeps the face animation running
void delayWithFace(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        updateFaceAnimation();
        delay(10);
    }
}

void setFaceWithMode(const String& faceName, FaceAnimMode mode) {
    currentFace      = faceName;
    currentFaceMode  = mode;
    animationFrame   = 0;
    updateFaceAnimation();
}

void enterIdle() {
    setFaceWithMode("stand", FACE_ANIM_LOOP);
}

/* ============================================================================
 * 9. SERVO STUB
 * ============================================================================
 * Inverse Kinematics + Bezier curve code will be merged here in the next
 * integration step. The signature must remain identical.
 * ============================================================================ */

void setServoAngle(uint8_t channel, int angle) {
    // IK / Bezier implementation to be added in next merge.
    // Example: pwm.setPWM(channel, 0, angleToPulse(angle));
    delayMicroseconds(500);
}

/* ============================================================================
 * 10. POSE & MOTION ROUTINES
 * ============================================================================ */

bool pressingCheck(String cmd, int ms) {
    delayWithFace(ms);
    if (currentCommand == cmd || currentCommand == "") return true;
    return false;
}

void runRestPose() {
    Serial.println(F("REST"));
    setFaceWithMode("rest", FACE_ANIM_BOOMERANG);
    setServoAngle(R1, 90);  setServoAngle(R2, 90);
    setServoAngle(L1, 90);  setServoAngle(L2, 90);
    setServoAngle(R4, 45);  setServoAngle(R3, 135);
    setServoAngle(L3, 45);  setServoAngle(L4, 135);
}

void runStandPose(int face = 1) {
    Serial.println(F("STAND"));
    if (face == 1) setFaceWithMode("stand", FACE_ANIM_ONCE);
    setServoAngle(R1, 135); setServoAngle(R2, 45);
    setServoAngle(L1, 45);  setServoAngle(L2, 135);
    setServoAngle(R4, 60);  setServoAngle(R3, 120);
    setServoAngle(L3, 60);  setServoAngle(L4, 120);
    if (face == 1) enterIdle();
}

void runWavePose() {
    Serial.println(F("WAVE"));
    setFaceWithMode("wave", FACE_ANIM_ONCE);
    runStandPose(0);
    delayWithFace(200);
    setServoAngle(R4, 90);  setServoAngle(L3, 135);
    setServoAngle(L2, 90);  setServoAngle(R1, 100);
    delayWithFace(200);
    for (int i = 0; i < 4; i++) {
        setServoAngle(L3, 135); delayWithFace(300);
        setServoAngle(L3, 90);  delayWithFace(300);
    }
    runStandPose(1);
    if (currentCommand == "wave") currentCommand = "";
}

void runDancePose() {
    Serial.println(F("DANCE"));
    setFaceWithMode("dance", FACE_ANIM_LOOP);
    setServoAngle(R1, 90);  setServoAngle(R2, 90);
    setServoAngle(L1, 90);  setServoAngle(L2, 90);
    setServoAngle(R4, 120); setServoAngle(R3, 120);
    setServoAngle(L3, 60);  setServoAngle(L4, 60);
    delayWithFace(300);
    for (int i = 0; i < 5; i++) {
        setServoAngle(R4, 90);  setServoAngle(R3, 90);
        setServoAngle(L3, 60);  setServoAngle(L4, 60);
        delayWithFace(300);
        setServoAngle(R4, 120); setServoAngle(R3, 120);
        setServoAngle(L3, 90);  setServoAngle(L4, 90);
        delayWithFace(300);
    }
    runStandPose(1);
    if (currentCommand == "dance") currentCommand = "";
}

void runPushupPose() {
    Serial.println(F("PUSHUP"));
    setFaceWithMode("pushup", FACE_ANIM_ONCE);
    runStandPose(0);
    delayWithFace(200);
    setServoAngle(L1, 30);  setServoAngle(R1, 150);
    setServoAngle(L3, 90);  setServoAngle(R3, 90);
    delayWithFace(500);
    for (int i = 0; i < 4; i++) {
        setServoAngle(L3, 45);  setServoAngle(R3, 135); delayWithFace(600);
        setServoAngle(L3, 90);  setServoAngle(R3, 90);  delayWithFace(500);
    }
    runStandPose(1);
    if (currentCommand == "pushup") currentCommand = "";
}

void runDeadPose() {
    Serial.println(F("DEAD"));
    runStandPose(0);
    setFaceWithMode("dead", FACE_ANIM_BOOMERANG);
    delayWithFace(200);
    setServoAngle(R3, 45);  setServoAngle(R4, 45);
    setServoAngle(L3, 135); setServoAngle(L4, 135);
    if (currentCommand == "dead") currentCommand = "";
}

void runWalkPose() {
    Serial.println(F("WALK FWD"));
    setFaceWithMode("walk", FACE_ANIM_ONCE);
    setServoAngle(R3, 120); setServoAngle(L3, 60);
    setServoAngle(R2, 90);  setServoAngle(L1, 45);
    if (!pressingCheck("forward", frameDelay)) return;
    for (int i = 0; i < walkCycles; i++) {
        setServoAngle(R3, 120); setServoAngle(L3, 30);
        if (!pressingCheck("forward", frameDelay)) return;
        setServoAngle(L4, 120); setServoAngle(L2, 90);
        setServoAngle(R4, 30);  setServoAngle(R1, 150);
        if (!pressingCheck("forward", frameDelay)) return;
        setServoAngle(R2, 45);  setServoAngle(L1, 90);
        if (!pressingCheck("forward", frameDelay)) return;
        setServoAngle(R4, 60);  setServoAngle(L4, 150);
        if (!pressingCheck("forward", frameDelay)) return;
        setServoAngle(R3, 150); setServoAngle(L3, 60);
        setServoAngle(R2, 90);  setServoAngle(L1, 30);
        if (!pressingCheck("forward", frameDelay)) return;
        setServoAngle(L2, 120); setServoAngle(R1, 90);
        if (!pressingCheck("forward", frameDelay)) return;
    }
    runStandPose(1);
}

/* ============================================================================
 * 11. MOOD / BEHAVIOR CONTROLLER
 * ============================================================================ */

void executeRobotMood(const String& mood) {
    if      (mood == "HAPPY"  || mood == "happy"   || mood == "highfive" || mood == "wave")
        runWavePose();
    else if (mood == "DANCE"  || mood == "dance")
        runDancePose();
    else if (mood == "ANGRY"  || mood == "angry")
        runPushupPose();
    else if (mood == "SLEEPY" || mood == "sleep"   || mood == "rest")
        runRestPose();
    else if (mood == "TIRED"  || mood == "dead")
        runDeadPose();
    else if (mood == "stand")
        runStandPose(1);
}

/* ============================================================================
 * 12. MQTT COMMAND HANDLERS
 * ============================================================================ */

static void handleMovement(const String& cmd) {
    /*
     * Future IK integration point:
     *   forward  -> locomotion.setVelocity(+V, 0)
     *   backward -> locomotion.setVelocity(-V, 0)
     *   left     -> locomotion.setYawRate(-W)
     *   right    -> locomotion.setYawRate(+W)
     *   stop     -> locomotion.halt()
     */
    if (cmd == "forward" || cmd == "backward" ||
        cmd == "left"    || cmd == "right"    || cmd == "stop") {
        currentCommand = cmd;
        logTag("MOVE", cmd.c_str());
        if (cmd == "forward") runWalkPose();
        // Other directions: stub — locomotion code to be added with IK merge
        if (cmd == "stop") {
            currentCommand = "";
            runStandPose(1);
        }
    } else {
        logTagF("MOVE", "Unknown command: %s", cmd.c_str());
    }
}

static void handleBehavior(const String& cmd) {
    if (cmd == "stand"   || cmd == "sleep"   || cmd == "highfive" ||
        cmd == "happy"   || cmd == "wave"    || cmd == "dance") {
        robotState.currentBehavior = cmd;
        currentCommand = cmd;
        logTag("BEHAVIOR", cmd.c_str());
        executeRobotMood(cmd);
    } else {
        logTagF("BEHAVIOR", "Unknown command: %s", cmd.c_str());
    }
}

static void handleTool(const String& cmd) {
    if      (cmd == "sprayer_on")     { pumpOn();       logTag("TOOL", "sprayer_on"); }
    else if (cmd == "sprayer_off")    { pumpOff();      logTag("TOOL", "sprayer_off"); }
    else if (cmd == "cutter_on"  ||
             cmd == "cutter_high")    { CutterHigh();   logTag("TOOL", "cutter_high"); }
    else if (cmd == "cutter_medium")  { CutterMedium(); logTag("TOOL", "cutter_medium"); }
    else if (cmd == "cutter_off")     { CutterOff();    logTag("TOOL", "cutter_off"); }
    else {
        logTagF("TOOL", "Unknown command: %s", cmd.c_str());
    }
}

/* ============================================================================
 * 13. MQTT CALLBACK — DISPATCHER
 * ============================================================================ */

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char buf[64];
    if (length >= sizeof(buf)) length = sizeof(buf) - 1;
    memcpy(buf, payload, length);
    buf[length] = '\0';

    String cmd = String(buf);
    cmd.trim();
    String tpc = String(topic);

    logTagF("MQTT", "RX %s : %s", tpc.c_str(), cmd.c_str());

    if      (tpc == TOPIC_MOVE)     handleMovement(cmd);
    else if (tpc == TOPIC_BEHAVIOR) handleBehavior(cmd);
    else if (tpc == TOPIC_TOOL)     handleTool(cmd);
    else    logTagF("MQTT", "Unhandled topic: %s", tpc.c_str());
}

/* ============================================================================
 * 14. MQTT MANAGER
 * ============================================================================ */

static void mqttBegin() {
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setKeepAlive(MQTT_KEEPALIVE_SEC);
    mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_MS / 1000);
    mqttClient.setBufferSize(512);
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
    bool s1 = mqttClient.subscribe(TOPIC_MOVE);
    bool s2 = mqttClient.subscribe(TOPIC_BEHAVIOR);
    bool s3 = mqttClient.subscribe(TOPIC_TOOL);
    logTagF("MQTT", "Subscribed move=%d behavior=%d tool=%d",
            s1 ? 1 : 0, s2 ? 1 : 0, s3 ? 1 : 0);
    return true;
}

static void mqttMaintain() {
    mqttClient.loop();

    if (mqttClient.connected()) {
        if (!robotState.mqttConnected) robotState.mqttConnected = true;
        return;
    }

    if (robotState.mqttConnected) {
        robotState.mqttConnected = false;
        logTag("MQTT", "Disconnected. Will retry...");
    }

    uint32_t now = millis();
    if (now - lastMqttRetryMs >= MQTT_RETRY_INTERVAL_MS) {
        lastMqttRetryMs = now;
        if (robotState.wifiConnected) mqttConnect();
        else logTag("MQTT", "Skipping reconnect - WiFi down");
    }
}

/* ============================================================================
 * 15. TELEMETRY PUBLISHER
 * ============================================================================ */

static void publishTelemetry() {
    StaticJsonDocument<TELEMETRY_JSON_SIZE> doc;
    doc["battery"] = robotState.batteryPercent;
    doc["wifi"]    = robotState.wifiConnected ? "connected" : "disconnected";
    doc["state"]   = robotState.currentBehavior;
    doc["sprayer"] = robotState.sprayerOn  ? "on" : "off";
    doc["cutter"]  = robotState.cutterOn   ? "on" : "off";
    doc["uptime"]  = (uint32_t)(millis() - robotState.bootTimeMs) / 1000UL;

    char payload[TELEMETRY_JSON_SIZE];
    size_t written = serializeJson(doc, payload, sizeof(payload));
    if (written == 0) { logTag("STATUS", "JSON serialization failed"); return; }

    bool ok = mqttClient.publish(TOPIC_STATUS, payload, false);
    if (ok) logTag("STATUS", "Published");
    else    logTagF("STATUS", "Publish failed, mqtt_state=%d", mqttClient.state());
}

/* ============================================================================
 * 16. ARDUINO LIFECYCLE
 * ============================================================================ */

void setup() {
    Serial.begin(115200);
    delay(150);
    Serial.println();
    Serial.println(F("==========================================================="));
    Serial.println(F(" Agricultural Spider Robot - ESP32 Combined Master v1.0   "));
    Serial.println(F("==========================================================="));

    robotState.bootTimeMs = millis();

    // --- OLED init ---
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed — check I2C wiring"));
        for (;;);   // Halt immediately on display failure
    }
    display.clearDisplay();

    // --- GPIO / Motor drivers init ---
    driversInit();

    // --- WiFi + MQTT init ---
    wifiBegin();
    mqttBegin();

    // --- Boot pose ---
    runStandPose(1);

    logTag("BOOT", "Initialization complete — waiting for MQTT commands");
}

void loop() {
    /* 1. Maintain WiFi (non-blocking) */
    wifiMaintain();

    /* 2. Maintain MQTT (non-blocking) */
    mqttMaintain();

    /* 3. Periodic telemetry publish (1 Hz, non-blocking) */
    uint32_t now = millis();
    if (robotState.mqttConnected && (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)) {
        lastTelemetryMs = now;
        publishTelemetry();
    }

    /* 4. Keep OLED face animation alive between commands */
    updateFaceAnimation();

    /* 5. Yield to lower-priority tasks (WDT-friendly) */
    yield();
}
