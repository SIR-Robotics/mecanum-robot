// ─── RoboArm 6-DOF Controller — main_new.cpp ────────────────────────────────
// Game-style web UI: dual XY thumbsticks + analog triggers + keyboard +
// browser Gamepad API. Engine inherited from new_case.cpp:
//   - 181-entry angle→PCA9685 counts LUT (no map() in hot path)
//   - Zero-alloc outgoing JSON via snprintf into static buffers
//   - 2-letter WS text protocol (new: JX for dual-axis jog in one frame)
//   - I2C mutex for cross-core safety, FreeRTOS queue for WS frames
//   - WebJog acceleration accumulator (smooth, frame-rate independent)
//
// Active build: flip platformio.ini build_src_filter to +<main_new.cpp>.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// ─── PCA9685 ────────────────────────────────────────────────────────────────
#define PCA9685_ADDR 0x40
#define PWM_FREQ_HZ  50
#define OSC_FREQ     27000000
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define I2C_SDA      21
#define I2C_SCL      22

// ─── WiFi (STA) ─────────────────────────────────────────────────────────────
const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";

// ─── Hardware joystick & buttons (ADC1 only) ────────────────────────────────
#define JOY_X_PIN     34
#define JOY_Y_PIN     35
#define JOY_SW_PIN    32
#define BTN_REC_PIN   33
#define BTN_PLAY_PIN  25
#define BTN_CLR_PIN   26
#define BTN_CYCLE_PIN 27

#define JOY_DEADZONE  300
#define JOY_SPEED     1.2f
#define JOY_INTERVAL  20
#define DEBOUNCE_MS   220

const uint8_t PAIR[3][2]   = { {0,1}, {2,3}, {4,5} };
const char*   PAIR_NAME[3] = { "Base+Shoulder", "Elbow+Wrist Pitch", "Wrist Roll+Gripper" };

// ─── Web jog ────────────────────────────────────────────────────────────────
#define WEB_JOG_INTERVAL 25
#define WEB_JOG_SPEED    2.0f

// ─── Recording ──────────────────────────────────────────────────────────────
#define MAX_POSES    50
#define PLAY_STEP_MS 1200

struct Pose { int a[6]; char label[20]; };

Pose     seq[MAX_POSES];
int      seqLen     = 0;
bool     isPlaying  = false;
bool     isCycling  = false;
int      playIdx    = 0;
uint32_t playNextMs = 0;

// ─── Joint table ────────────────────────────────────────────────────────────
struct Joint {
    const char* name;
    uint8_t     ch;
    int         lo, hi;
    int         home;
    int         cur;
    bool        invert;
};

Joint joints[6] = {
    { "Base",        5,   0, 180,  90,  90, false },
    { "Shoulder",    0,  30, 150,  90,  90, true  },
    { "Elbow",       1,   0, 135,  90,  90, false },
    { "Wrist Pitch", 2,   0, 180,  90,  90, false },
    { "Wrist Roll",  3,   0, 180,  90,  90, false },
    { "Gripper",     4,   0,  90,  45,  45, false },
};

const int POSE_HOME [6] = { 90, 90, 90, 90, 90, 45 };
const int POSE_READY[6] = { 90, 60, 90, 90, 90, 45 };
const int POSE_PICK [6] = { 90, 45, 45, 90, 90,  0 };
const int POSE_PLACE[6] = { 90, 45, 45, 90, 90, 90 };

// ─── Globals ────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pca9685(PCA9685_ADDR);
AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");
Preferences     prefs;
SemaphoreHandle_t servoMutex;

float    joyF[6];
uint8_t  joyMode    = 0;
int      joyXCenter = 2048;
int      joyYCenter = 2048;
bool     joyActive  = false;
uint32_t joyLastMs  = 0;
uint32_t lastBroadMs = 0;

int      webJog[6]    = {0,0,0,0,0,0};
float    webJogAcc[6] = {0,0,0,0,0,0};
uint32_t lastWebJogMs = 0;
bool     webJogActive = false;

uint32_t lastSW=0, lastRec=0, lastPlay=0, lastClr=0, lastCycle=0;

struct WsMsg { char buf[96]; };
QueueHandle_t wsQueue;
bool pendingBroadcast = false;

uint16_t pwmTable[181];

static char statusBuf[220];
static char posesBuf[3000];

// ─── Servo ──────────────────────────────────────────────────────────────────
inline uint16_t toCounts(int a) {
    return pwmTable[a < 0 ? 0 : (a > 180 ? 180 : a)];
}

void setServo(uint8_t i, int angle) {
    angle = constrain(angle, joints[i].lo, joints[i].hi);
    joints[i].cur = angle;
    joyF[i]       = (float)angle;
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pca9685.setPWM(joints[i].ch, 0, toCounts(phys));
        xSemaphoreGive(servoMutex);
    }
}

inline void sendPWM(uint8_t i, int angle) {
    joints[i].cur = angle;
    int phys = joints[i].invert ? (joints[i].lo + joints[i].hi - angle) : angle;
    pca9685.setPWM(joints[i].ch, 0, toCounts(phys));
}

void smoothMove(uint8_t i, int target, uint8_t steps = 40, uint8_t ms = 15) {
    int start = joints[i].cur;
    target = constrain(target, joints[i].lo, joints[i].hi);
    for (uint8_t s = 1; s <= steps; s++) {
        setServo(i, start + (target - start) * s / steps);
        delay(ms);
    }
}

void moveToHome()                { for (int i=0;i<6;i++) setServo(i, joints[i].home); }
void applyPreset(const int p[6]) { for (int i=0;i<6;i++) smoothMove(i, p[i]); }

// ─── JSON (no heap) ─────────────────────────────────────────────────────────
const char* buildStatus() {
    snprintf(statusBuf, sizeof(statusBuf),
        "{\"t\":\"s\",\"j\":[%d,%d,%d,%d,%d,%d],"
        "\"m\":%u,\"l\":%d,\"p\":%d,\"c\":%d,\"i\":%d,\"r\":%d}",
        joints[0].cur, joints[1].cur, joints[2].cur,
        joints[3].cur, joints[4].cur, joints[5].cur,
        (unsigned)joyMode, seqLen,
        isPlaying ? 1 : 0, isCycling ? 1 : 0, playIdx,
        (int)WiFi.RSSI());
    return statusBuf;
}

const char* buildPoses() {
    char* p   = posesBuf;
    char* end = posesBuf + sizeof(posesBuf);
    p += snprintf(p, end - p, "{\"t\":\"p\",\"c\":%d,\"i\":[", seqLen);
    for (int i = 0; i < seqLen && (end - p) > 80; i++) {
        if (i) *p++ = ',';
        p += snprintf(p, end - p,
            "{\"a\":[%d,%d,%d,%d,%d,%d],\"n\":\"%s\"}",
            seq[i].a[0], seq[i].a[1], seq[i].a[2],
            seq[i].a[3], seq[i].a[4], seq[i].a[5],
            seq[i].label);
    }
    if ((end - p) >= 3) { *p++ = ']'; *p++ = '}'; *p = '\0'; }
    return posesBuf;
}

inline void broadcastStatus() { ws.textAll(buildStatus()); }
inline void broadcastPoses()  { ws.textAll(buildPoses());  }

// ─── Recording ──────────────────────────────────────────────────────────────
void recordPose() {
    if (seqLen >= MAX_POSES) { Serial.println("[REC] Full"); return; }
    for (int i = 0; i < 6; i++) seq[seqLen].a[i] = joints[i].cur;
    snprintf(seq[seqLen].label, sizeof(seq[seqLen].label), "Pose %d", seqLen + 1);
    seqLen++;
    Serial.printf("[REC] %d total\n", seqLen);
    pendingBroadcast = true;
}

void renamePose(int idx, const char* name) {
    if (idx < 0 || idx >= seqLen) return;
    strncpy(seq[idx].label, name, sizeof(seq[idx].label) - 1);
    seq[idx].label[sizeof(seq[idx].label) - 1] = '\0';
}

void startPlayback() {
    if (!seqLen) { Serial.println("[PLAY] Empty"); return; }
    isCycling = false; isPlaying = true;
    playIdx = 0; playNextMs = millis();
    pendingBroadcast = true;
}
void stopPlayback() { isPlaying = false; isCycling = false; pendingBroadcast = true; }

void startCycle() {
    if (!seqLen) { Serial.println("[CYCLE] Empty"); return; }
    isPlaying = false; isCycling = true;
    playIdx = 0; playNextMs = millis();
    pendingBroadcast = true;
}
void stopCycle() { isCycling = false; pendingBroadcast = true; }

void clearRecording() {
    seqLen = 0; isPlaying = false; isCycling = false; pendingBroadcast = true;
    Serial.println("[REC] Cleared");
}

void saveToFlash() {
    prefs.begin("roboarm", false);
    prefs.putInt("len", seqLen);
    if (seqLen > 0) prefs.putBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Saved %d\n", seqLen);
}

void loadFromFlash() {
    prefs.begin("roboarm", true);
    seqLen = prefs.getInt("len", 0);
    if (seqLen > 0) prefs.getBytes("seq", seq, seqLen * sizeof(Pose));
    prefs.end();
    Serial.printf("[FLASH] Loaded %d\n", seqLen);
    pendingBroadcast = true;
}

// ─── HW joystick ────────────────────────────────────────────────────────────
void calibrateJoystick() {
    long xs = 0, ys = 0;
    const int N = 64;
    for (int i = 0; i < N; i++) {
        xs += analogRead(JOY_X_PIN);
        ys += analogRead(JOY_Y_PIN);
        delay(5);
    }
    joyXCenter = xs / N;
    joyYCenter = ys / N;
    Serial.printf("[JOY] Center X=%d Y=%d\n", joyXCenter, joyYCenter);
}

void processJoystick() {
    if (millis() - joyLastMs < JOY_INTERVAL) return;
    joyLastMs = millis();

    int xRaw = analogRead(JOY_X_PIN);
    int yRaw = analogRead(JOY_Y_PIN);

    float xD = 0, yD = 0;
    if (abs(xRaw - joyXCenter) > JOY_DEADZONE)
        xD = (xRaw - joyXCenter) / (float)joyXCenter * JOY_SPEED;
    if (abs(yRaw - joyYCenter) > JOY_DEADZONE)
        yD = (yRaw - joyYCenter) / (float)joyYCenter * JOY_SPEED;

    uint8_t jA = PAIR[joyMode][0];
    uint8_t jB = PAIR[joyMode][1];
    joyActive = (xD != 0.0f || yD != 0.0f);

    if (xD != 0.0f) {
        joyF[jA] = constrain(joyF[jA] + xD, (float)joints[jA].lo, (float)joints[jA].hi);
        int a = (int)joyF[jA];
        if (a != joints[jA].cur) sendPWM(jA, a);
    }
    if (yD != 0.0f) {
        joyF[jB] = constrain(joyF[jB] + yD, (float)joints[jB].lo, (float)joints[jB].hi);
        int b = (int)joyF[jB];
        if (b != joints[jB].cur) sendPWM(jB, b);
    }
}

bool btnPressed(uint8_t pin, uint32_t& last) {
    if (digitalRead(pin) == LOW && millis() - last > DEBOUNCE_MS) {
        last = millis(); return true;
    }
    return false;
}

void processButtons() {
    if (btnPressed(JOY_SW_PIN,    lastSW))    { joyMode=(joyMode+1)%3; Serial.printf("[JOY] %s\n",PAIR_NAME[joyMode]); pendingBroadcast=true; }
    if (btnPressed(BTN_REC_PIN,   lastRec))   recordPose();
    if (btnPressed(BTN_PLAY_PIN,  lastPlay))  isPlaying ? stopPlayback() : startPlayback();
    if (btnPressed(BTN_CLR_PIN,   lastClr))   clearRecording();
    if (btnPressed(BTN_CYCLE_PIN, lastCycle)) isCycling ? stopCycle() : startCycle();
}

// ─── Web jog ────────────────────────────────────────────────────────────────
void processWebJog() {
    if (isPlaying || isCycling) { webJogActive = false; return; }
    if (millis() - lastWebJogMs < WEB_JOG_INTERVAL) return;
    lastWebJogMs = millis();

    bool moved = false, anyActive = false;
    for (int j = 0; j < 6; j++) {
        if (webJog[j] == 0) { webJogAcc[j] = 0; continue; }
        anyActive = true;
        webJogAcc[j] += (webJog[j] / 100.0f) * WEB_JOG_SPEED;
        int delta = (int)webJogAcc[j];
        if (delta == 0) continue;
        int target = constrain(joints[j].cur + delta, joints[j].lo, joints[j].hi);
        if (target == joints[j].cur) { webJogAcc[j] = 0; continue; }
        webJogAcc[j] -= (target - joints[j].cur);
        sendPWM(j, target);
        joyF[j] = (float)target;
        moved = true;
    }
    webJogActive = anyActive;
    if (moved) pendingBroadcast = true;
}

void processPlayback() {
    if (!isPlaying && !isCycling) return;
    if (millis() < playNextMs)    return;

    if (playIdx >= seqLen) {
        if (isCycling) { playIdx = 0; }
        else {
            isPlaying = false; playIdx = 0;
            Serial.println("[PLAY] Done");
            pendingBroadcast = true;
            return;
        }
    }

    for (int i = 0; i < 6; i++) setServo(i, seq[playIdx].a[i]);
    Serial.printf("[%s] %d/%d \"%s\"\n",
        isCycling ? "CYC" : "PLAY", playIdx + 1, seqLen, seq[playIdx].label);
    playIdx++;
    playNextMs = millis() + PLAY_STEP_MS;
    pendingBroadcast = true;
}

// ─── WS protocol ────────────────────────────────────────────────────────────
// Tags:
//   JG:j:v             single-axis jog  (-100..100)
//   JX:j1:v1:j2:v2     dual-axis jog (one XY stick → 2 joints in one frame)
//   SV:j:a             absolute servo set
//   PR:name            preset (home/ready/pick/place)
//   MD:m               HW joystick pair (0..2)
//   RC / PY / ST / CY / CL / SA / LD       record/play/stop/cycle/clear/save/load
//   RN:p:name          rename pose
//   GT:p               goto pose
#define TAG(a,b) ((uint16_t)(((a)<<8) | (b)))

void processWsCmd(char* msg) {
    if (msg[0] == 0 || msg[1] == 0) return;
    uint16_t tag = TAG((uint8_t)msg[0], (uint8_t)msg[1]);

    char* args[4] = { nullptr, nullptr, nullptr, nullptr };
    if (msg[2] == ':') {
        char* s = msg + 3;
        args[0] = s;
        for (int i = 1; i < 4 && (s = strchr(s, ':')) != nullptr; i++) {
            *s++ = '\0';
            args[i] = s;
        }
    }

    switch (tag) {
        case TAG('J','G'): {
            if (!args[0] || !args[1]) break;
            int j = atoi(args[0]), v = atoi(args[1]);
            if (j >= 0 && j < 6) webJog[j] = constrain(v, -100, 100);
            break;
        }
        case TAG('J','X'): {
            if (!args[0] || !args[1] || !args[2] || !args[3]) break;
            int j1 = atoi(args[0]), v1 = atoi(args[1]);
            int j2 = atoi(args[2]), v2 = atoi(args[3]);
            if (j1 >= 0 && j1 < 6) webJog[j1] = constrain(v1, -100, 100);
            if (j2 >= 0 && j2 < 6) webJog[j2] = constrain(v2, -100, 100);
            break;
        }
        case TAG('S','V'): {
            if (!args[0] || !args[1]) break;
            int j = atoi(args[0]), a = atoi(args[1]);
            if (j >= 0 && j < 6 && a >= 0) { setServo(j, a); pendingBroadcast = true; }
            break;
        }
        case TAG('P','R'): {
            if (!args[0]) break;
            const int* p = nullptr;
            if      (!strcmp(args[0], "home"))  p = POSE_HOME;
            else if (!strcmp(args[0], "ready")) p = POSE_READY;
            else if (!strcmp(args[0], "pick"))  p = POSE_PICK;
            else if (!strcmp(args[0], "place")) p = POSE_PLACE;
            if (p) { for (int i = 0; i < 6; i++) setServo(i, p[i]); pendingBroadcast = true; }
            break;
        }
        case TAG('M','D'): {
            if (!args[0]) break;
            int m = atoi(args[0]);
            if (m >= 0 && m < 3) { joyMode = (uint8_t)m; pendingBroadcast = true; }
            break;
        }
        case TAG('R','C'): recordPose(); broadcastPoses(); break;
        case TAG('R','N'): {
            if (!args[0] || !args[1] || !*args[1]) break;
            int p = atoi(args[0]);
            if (p >= 0 && p < seqLen) {
                renamePose(p, args[1]);
                broadcastPoses();
                saveToFlash();
            }
            break;
        }
        case TAG('G','T'): {
            if (!args[0]) break;
            int p = atoi(args[0]);
            if (p >= 0 && p < seqLen) {
                for (int i = 0; i < 6; i++) setServo(i, seq[p].a[i]);
                pendingBroadcast = true;
            }
            break;
        }
        case TAG('P','Y'): startPlayback(); break;
        case TAG('S','T'): stopPlayback(); break;
        case TAG('C','Y'): isCycling ? stopCycle() : startCycle(); break;
        case TAG('C','L'): clearRecording(); broadcastPoses(); break;
        case TAG('S','A'): saveToFlash(); break;
        case TAG('L','D'): loadFromFlash(); broadcastPoses(); break;
    }
}

void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        client->text(buildStatus());
        client->text(buildPoses());
    } else if (type == WS_EVT_DISCONNECT) {
        for (int j = 0; j < 6; j++) webJog[j] = 0;
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            WsMsg m; size_t n = min(len, sizeof(m.buf) - 1);
            memcpy(m.buf, data, n); m.buf[n] = '\0';
            xQueueSend(wsQueue, &m, 0);
        }
    }
}

// ─── Web UI ─────────────────────────────────────────────────────────────────
// Game pad layout:
//   • Left thumbstick (XY) → Base / Shoulder
//   • Right thumbstick (XY) → Elbow / Wrist Pitch
//   • Two vertical triggers → Wrist Roll, Gripper
//   • Keyboard: WASD (left stick), IJKL (right stick), Q/E (roll), Z/X (grip),
//               Space=REC, P=PLAY, S=STOP, C=CYCLE, H=HOME
//   • Browser Gamepad API: real Xbox/PS controller mapped to the same controls
const char HTML_PAGE[] = R"raw(<!DOCTYPE html><html lang=en><head><meta charset=UTF-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no"><title>RoboArm</title><style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;user-select:none}
:root{--bg:#0b0f1a;--p1:#16213e;--p2:#0f1f3a;--ac:#e94560;--ac2:#27ae60;--ac3:#4fc3f7;--tx:#eef;--mu:#666}
body{font-family:Segoe UI,system-ui,sans-serif;background:radial-gradient(circle at 50% 0%,#1a2540 0,var(--bg) 60%);color:var(--tx);min-height:100vh;padding:10px;overscroll-behavior:none}
.hud{display:flex;align-items:center;gap:10px;background:linear-gradient(90deg,#16213e,#0f1f3a);border-radius:10px;padding:8px 12px;margin-bottom:10px;border:1px solid #2a3a5e;font-size:.78rem}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:#444;box-shadow:0 0 0 0 #444;transition:background .3s,box-shadow .3s}
.dot.ok{background:var(--ac2);box-shadow:0 0 8px var(--ac2)}
.dot.err{background:var(--ac);box-shadow:0 0 8px var(--ac)}
.hud .tt{color:var(--ac);font-weight:700;letter-spacing:1px}
.hud .gap{flex:1}
.hud .pill{background:var(--p2);padding:3px 8px;border-radius:10px;color:var(--mu);font-size:.7rem}
.hud .pill b{color:var(--ac3);font-weight:700}
.jr{display:grid;grid-template-columns:repeat(6,1fr);gap:6px;margin-bottom:10px}
.jc{background:var(--p2);border-radius:8px;padding:7px 4px;text-align:center;border:1px solid transparent;transition:border-color .15s,box-shadow .15s}
.jc.act{border-color:var(--ac);box-shadow:0 0 10px rgba(233,69,96,.3)}
.jc .l{font-size:.6rem;color:#789;letter-spacing:1px}
.jc .v{font-size:1.05rem;color:var(--ac);font-weight:800;font-family:Consolas,monospace}
.pad{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:10px}
.stk{background:linear-gradient(180deg,var(--p1),var(--p2));border-radius:14px;padding:14px;border:1px solid #2a3a5e;position:relative}
.stk .ttl{font-size:.62rem;color:var(--ac3);letter-spacing:2px;font-weight:700;margin-bottom:8px;text-transform:uppercase;text-align:center}
.stk .sub{font-size:.6rem;color:var(--mu);letter-spacing:1px;text-align:center;margin-bottom:8px}
.spad{width:100%;aspect-ratio:1/1;max-width:230px;margin:0 auto;background:radial-gradient(circle at 50% 50%,#0a1428 0,#040814 100%);border-radius:50%;position:relative;touch-action:none;border:2px solid #2a3a5e;box-shadow:inset 0 0 25px rgba(0,0,0,.6)}
.spad::before{content:"";position:absolute;left:50%;top:0;width:1px;height:100%;background:rgba(79,195,247,.08)}
.spad::after{content:"";position:absolute;top:50%;left:0;height:1px;width:100%;background:rgba(79,195,247,.08)}
.knob{width:34%;aspect-ratio:1/1;background:radial-gradient(circle at 35% 30%,#ff6b88,var(--ac));border-radius:50%;position:absolute;left:33%;top:33%;box-shadow:0 4px 14px rgba(233,69,96,.45),inset 0 -3px 8px rgba(0,0,0,.35);transition:background .15s,box-shadow .15s,transform .05s}
.spad.act .knob{background:radial-gradient(circle at 35% 30%,#5be095,var(--ac2));box-shadow:0 4px 18px rgba(39,174,96,.55),inset 0 -3px 8px rgba(0,0,0,.35)}
.tg{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:10px}
.tw{background:linear-gradient(180deg,var(--p1),var(--p2));border-radius:14px;padding:12px 14px;border:1px solid #2a3a5e}
.tw .ttl{font-size:.62rem;color:var(--ac3);letter-spacing:2px;font-weight:700;margin-bottom:8px;text-transform:uppercase;text-align:center}
.trk{width:60px;height:130px;margin:0 auto;background:radial-gradient(circle at 50% 50%,#0a1428,#040814);border-radius:30px;position:relative;touch-action:none;border:2px solid #2a3a5e;overflow:hidden;box-shadow:inset 0 0 15px rgba(0,0,0,.6)}
.tnb{width:46px;height:46px;background:radial-gradient(circle at 35% 30%,#ff6b88,var(--ac));border-radius:50%;position:absolute;left:5px;top:40px;box-shadow:0 3px 10px rgba(233,69,96,.4),inset 0 -3px 6px rgba(0,0,0,.35);transition:background .15s}
.trk.act .tnb{background:radial-gradient(circle at 35% 30%,#5be095,var(--ac2));box-shadow:0 3px 14px rgba(39,174,96,.5),inset 0 -3px 6px rgba(0,0,0,.35)}
.p{background:var(--p1);border-radius:12px;padding:12px;margin-bottom:10px;border:1px solid #2a3a5e}
.p h2{font-size:.62rem;color:var(--ac);margin-bottom:10px;text-transform:uppercase;letter-spacing:3px;font-weight:700}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:7px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:7px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:7px}
button{padding:11px 6px;border:0;border-radius:8px;background:linear-gradient(180deg,#1a3060,#0f3460);color:var(--tx);font-size:.78rem;cursor:pointer;font-weight:700;line-height:1.3;transition:transform .08s,background .15s,box-shadow .15s;border:1px solid #2a4070}
button:hover{background:linear-gradient(180deg,#3a1730,var(--ac));border-color:var(--ac)}
button:active{transform:scale(.96)}
button.on{background:linear-gradient(180deg,#3a1730,var(--ac));border-color:var(--ac);box-shadow:0 0 10px rgba(233,69,96,.4)}
button.red{background:linear-gradient(180deg,#3a0f0f,#5a1a1a);border-color:#5a2020}
button.red:hover{background:linear-gradient(180deg,#5a1a1a,#c0392b);border-color:#c0392b}
button.grn{background:linear-gradient(180deg,#0f3a1a,#1a5a2a);border-color:#205020}
button.grn:hover{background:linear-gradient(180deg,#1a5a2a,#27ae60);border-color:var(--ac2)}
button.amb{background:linear-gradient(180deg,#3a2a00,#5a4400);font-size:.88rem;letter-spacing:.5px;border-color:#5a4400}
button.amb:hover{background:linear-gradient(180deg,#5a4400,#e6a817);border-color:#e6a817}
button.amb.on{background:linear-gradient(180deg,#e6a817,#c08810);color:#111;border-color:#e6a817;box-shadow:0 0 10px rgba(230,168,23,.5)}
.mb{background:var(--p2);border-radius:6px;padding:7px 11px;margin-bottom:8px;font-size:.74rem;color:var(--mu)}
.mb b{color:var(--ac)}
.bd{display:inline-block;background:var(--ac);color:#fff;border-radius:10px;font-size:.62rem;padding:1px 7px;margin-left:5px;vertical-align:middle;font-weight:700}
.pl{max-height:170px;overflow-y:auto;margin-top:10px}
.pi{display:flex;align-items:center;gap:7px;padding:6px 9px;border-radius:6px;margin-bottom:3px;background:var(--p2);border:1px solid transparent;transition:border-color .12s}
.pi:hover{border-color:#2a3a5e}
.pi.cur{border-color:var(--ac2);box-shadow:0 0 8px rgba(39,174,96,.3)}
.pnn{color:var(--ac);font-weight:800;font-size:.76rem;min-width:26px}
.pll{color:#dde;font-size:.78rem;cursor:pointer;flex:1;padding:2px 4px;border-radius:4px}
.pll:hover{background:#1a3060;color:var(--ac)}
.pa{color:#556;font-size:.64rem;cursor:pointer;font-family:Consolas,monospace}
.pa:hover{color:var(--ac3)}
.ri{background:var(--p2);color:var(--tx);border:1px solid var(--ac);border-radius:4px;padding:2px 5px;font-size:.78rem;width:120px;outline:none}
#toast{position:fixed;left:50%;transform:translate(-50%,0);bottom:18px;background:rgba(20,28,50,.95);border:1px solid var(--ac);color:var(--tx);padding:9px 16px;border-radius:22px;font-size:.78rem;font-weight:600;opacity:0;pointer-events:none;transition:opacity .25s,transform .25s;box-shadow:0 4px 18px rgba(0,0,0,.5);z-index:99}
#toast.show{opacity:1;transform:translate(-50%,-6px)}
.kbd{font-size:.62rem;color:#456;margin-top:5px;text-align:center;letter-spacing:1px}
.kbd kbd{background:#0a1428;border:1px solid #2a3a5e;border-radius:3px;padding:1px 5px;margin:0 1px;font-family:Consolas,monospace;color:var(--ac3)}
@media(max-width:520px){.jr{grid-template-columns:repeat(3,1fr)}.jc .v{font-size:.95rem}.spad{max-width:180px}}
</style></head><body>
<div class=hud><span class=dot id=dt></span><span class=tt>ROBOARM</span><span class=gap></span><span class=pill id=ip>--</span><span class=pill>RSSI <b id=rs>--</b></span><span class=pill id=gpd style=display:none>&#127918; <b>GP</b></span></div>
<div class=jr id=jr></div>
<div class=pad>
 <div class=stk><div class=ttl>LEFT STICK</div><div class=sub>BASE / SHOULDER</div><div class=spad id=sL data-jx=0 data-jy=1><div class=knob></div></div><div class=kbd><kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd></div></div>
 <div class=stk><div class=ttl>RIGHT STICK</div><div class=sub>ELBOW / WRIST P.</div><div class=spad id=sR data-jx=2 data-jy=3><div class=knob></div></div><div class=kbd><kbd>I</kbd><kbd>J</kbd><kbd>K</kbd><kbd>L</kbd></div></div>
</div>
<div class=tg>
 <div class=tw><div class=ttl>WRIST ROLL</div><div class=trk id=tR data-j=4><div class=tnb></div></div><div class=kbd><kbd>Q</kbd><kbd>E</kbd></div></div>
 <div class=tw><div class=ttl>GRIPPER</div><div class=trk id=tG data-j=5><div class=tnb></div></div><div class=kbd><kbd>Z</kbd><kbd>X</kbd></div></div>
</div>
<div class=p><h2>Presets</h2><div class=g4>
 <button onclick="w('PR:home');tt('Home')">&#127968; Home</button>
 <button onclick="w('PR:ready');tt('Ready')">&#9889; Ready</button>
 <button onclick="w('PR:pick');tt('Pick')">&#129693; Pick</button>
 <button onclick="w('PR:place');tt('Place')">&#128230; Place</button>
</div></div>
<div class=p><h2>Recording <span class=bd id=rc>0</span></h2>
 <div class=g4 style=margin-bottom:7px>
  <button class=grn onclick="w('RC');tt('Recorded')">&#9679; REC</button>
  <button id=bp onclick="w('PY')">&#9654; Play</button>
  <button class=red onclick="w('ST')">&#9646;&#9646; Stop</button>
  <button class=red onclick="dc()">&#128465; Clear</button>
 </div>
 <button id=bc class=amb style=width:100%;margin-bottom:7px onclick="w('CY')">&#9654;&#9654; CYCLE LOOP &mdash; OFF</button>
 <div class=g2>
  <button onclick="w('SA');tt('Saved')">&#128190; Save</button>
  <button onclick="w('LD');tt('Loading')">&#128228; Load</button>
 </div>
 <div class=pl id=pls></div>
 <div class=kbd style=margin-top:8px><kbd>SPACE</kbd>rec <kbd>P</kbd>play <kbd>O</kbd>stop <kbd>C</kbd>cycle <kbd>H</kbd>home</div>
</div>
<div id=toast></div>
<script>
const JD=[{n:'BASE',k:'B',mn:0,mx:180,hm:90},{n:'SHOULDER',k:'S',mn:30,mx:150,hm:90},{n:'ELBOW',k:'E',mn:0,mx:135,hm:90},{n:'WRIST P',k:'WP',mn:0,mx:180,hm:90},{n:'WRIST R',k:'WR',mn:0,mx:180,hm:90},{n:'GRIPPER',k:'G',mn:0,mx:90,hm:45}];
let ps=[],s,rT,lastA=[90,90,90,90,90,45],lastPlayIdx=-1;

// Joint chips
const jr=document.getElementById('jr');
JD.forEach((j,i)=>jr.insertAdjacentHTML('beforeend',`<div class=jc id=jc${i}><div class=l>${j.k}</div><div class=v id=v${i}>--</div></div>`));

// ── WebSocket
function cn(){
 s=new WebSocket(`ws://${location.hostname}/ws`);
 s.onopen=()=>{sd(1);tt('Connected');clearTimeout(rT)};
 s.onclose=()=>{sd(0);tt('Disconnected');rT=setTimeout(cn,2500)};
 s.onerror=()=>s.close();
 s.onmessage=(e)=>{const d=JSON.parse(e.data);if(d.t==='s')aS(d);else if(d.t==='p')aP(d)};
}
function w(x){if(s&&s.readyState===1)s.send(x)}
function sd(o){const d=document.getElementById('dt');d.classList.toggle('ok',!!o);d.classList.toggle('err',!o)}

// ── Toast
let tID=0;
function tt(m){const e=document.getElementById('toast');e.textContent=m;e.classList.add('show');clearTimeout(tID);tID=setTimeout(()=>e.classList.remove('show'),1500)}

// ── Apply status / poses
function aS(d){
 d.j.forEach((a,i)=>{
  document.getElementById('v'+i).textContent=a+'°';
  const c=document.getElementById('jc'+i);
  if(a!==lastA[i]){c.classList.add('act');clearTimeout(c._t);c._t=setTimeout(()=>c.classList.remove('act'),250)}
  lastA[i]=a;
 });
 document.getElementById('rc').textContent=d.l;
 document.getElementById('bp').classList.toggle('on',!!d.p);
 const cb=document.getElementById('bc');cb.classList.toggle('on',!!d.c);
 cb.innerHTML=d.c?'■ CYCLE LOOP &mdash; ON':'&#9654;&#9654; CYCLE LOOP &mdash; OFF';
 if(typeof d.r==='number')document.getElementById('rs').textContent=d.r+' dBm';
 if(d.c||d.p){
  if(d.i!==lastPlayIdx){lastPlayIdx=d.i;document.querySelectorAll('.pi').forEach((p,k)=>p.classList.toggle('cur',k===d.i-1))}
 } else if(lastPlayIdx!==-1){lastPlayIdx=-1;document.querySelectorAll('.pi').forEach(p=>p.classList.remove('cur'))}
}
function aP(d){ps=d.i||[];document.getElementById('rc').textContent=d.c;rP()}
function eh(x){return x.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;')}
function rP(){
 const e=document.getElementById('pls');e.innerHTML='';
 ps.forEach((p,i)=>{
  const d=document.createElement('div');d.className='pi';
  d.innerHTML=`<span class=pnn>#${i+1}</span><span class=pll id=pl${i}>${eh(p.n)}</span><span class=pa title="Go to pose">${p.a.map((v,k)=>JD[k].k+v).join(' ')}</span>`;
  d.querySelector('.pll').onclick=()=>sR(i);
  d.querySelector('.pa').onclick=()=>{w('GT:'+i);tt('Go #'+(i+1))};
  e.appendChild(d);
 });
}
function sR(i){
 const e=document.getElementById('pl'+i),o=ps[i].n;
 e.innerHTML=`<input class=ri type=text value="${eh(o)}" maxlength=19>`;
 const I=e.querySelector('input');I.focus();I.select();
 I.onblur=()=>{const v=I.value.trim().replace(/:/g,' ');if(v&&v!==o){w('RN:'+i+':'+v);tt('Renamed')}else e.textContent=o};
 I.onkeydown=ev=>{if(ev.key==='Enter')I.blur();if(ev.key==='Escape'){I.value=o;I.blur()}ev.stopPropagation()};
}

// ── XY thumbsticks
const sticks=[];
function bindStick(el){
 const jx=+el.dataset.jx,jy=+el.dataset.jy;
 const knob=el.querySelector('.knob');
 const st={el,knob,jx,jy,vx:0,vy:0,lvx:0,lvy:0,active:false,kbx:0,kby:0,gpx:0,gpy:0};
 function send(){w('JX:'+jx+':'+st.vx+':'+jy+':'+(-st.vy))}
 st.send=send;
 function compose(){
  const cx=st.dx||st.kbx||st.gpx, cy=st.dy||st.kby||st.gpy;
  st.vx=Math.round(cx*100);st.vy=Math.round(cy*100);
  const r=el.getBoundingClientRect(),k=knob.getBoundingClientRect();
  const R=(r.width-k.width)/2;
  knob.style.left=`calc(33% + ${cx*R}px)`;knob.style.top=`calc(33% + ${cy*R}px)`;
  const on=Math.abs(st.vx)>4||Math.abs(st.vy)>4;
  el.classList.toggle('act',on);st.active=on;
 }
 st.compose=compose;
 function start(e){st.dragging=true;e.preventDefault()}
 function end(){if(!st.dragging)return;st.dragging=false;st.dx=0;st.dy=0;compose();send()}
 function move(e){
  if(!st.dragging)return;
  const r=el.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;
  const px=e.touches?e.touches[0].clientX:e.clientX;
  const py=e.touches?e.touches[0].clientY:e.clientY;
  let dx=(px-cx)/(r.width/2),dy=(py-cy)/(r.height/2);
  const mag=Math.hypot(dx,dy);if(mag>1){dx/=mag;dy/=mag}
  st.dx=dx;st.dy=dy;compose();
 }
 st._move=move;st._end=end;
 el.addEventListener('mousedown',start);el.addEventListener('touchstart',start,{passive:false});
 sticks.push(st);
}
bindStick(document.getElementById('sL'));
bindStick(document.getElementById('sR'));
addEventListener('mousemove',e=>sticks.forEach(s=>s._move(e)));
addEventListener('touchmove',e=>sticks.forEach(s=>s._move(e)),{passive:false});
addEventListener('mouseup',()=>sticks.forEach(s=>s._end()));
addEventListener('touchend',()=>sticks.forEach(s=>s._end()));

// ── Triggers (vertical)
const trigs=[];
function bindTrig(el){
 const j=+el.dataset.j,nb=el.querySelector('.tnb');
 const tg={el,nb,j,v:0,lv:0,kb:0,gp:0,L:0};
 function send(){w('JG:'+j+':'+tg.v)}
 tg.send=send;
 function pos(){
  const cy=tg.dy!==undefined?tg.dy:(tg.kb||tg.gp);
  tg.v=Math.round(cy*100);
  nb.style.top=(40-cy*38)+'px';
  const on=Math.abs(tg.v)>4;el.classList.toggle('act',on);
 }
 tg.pos=pos;
 function start(e){tg.dragging=true;e.preventDefault()}
 function end(){if(!tg.dragging)return;tg.dragging=false;tg.dy=undefined;pos();send()}
 function move(e){
  if(!tg.dragging)return;
  const r=el.getBoundingClientRect(),cy=e.touches?e.touches[0].clientY:e.clientY;
  let y=cy-(r.top+r.height/2);y=Math.max(-46,Math.min(46,y));
  tg.dy=-y/46;pos();
 }
 tg._move=move;tg._end=end;
 el.addEventListener('mousedown',start);el.addEventListener('touchstart',start,{passive:false});
 trigs.push(tg);
}
bindTrig(document.getElementById('tR'));
bindTrig(document.getElementById('tG'));
addEventListener('mousemove',e=>trigs.forEach(t=>t._move(e)));
addEventListener('touchmove',e=>trigs.forEach(t=>t._move(e)),{passive:false});
addEventListener('mouseup',()=>trigs.forEach(t=>t._end()));
addEventListener('touchend',()=>trigs.forEach(t=>t._end()));

// ── Keyboard
const keys={};
addEventListener('keydown',e=>{
 if(document.activeElement.tagName==='INPUT')return;
 const k=e.key.toLowerCase();
 if(keys[k])return;          // ignore OS key-repeat
 keys[k]=true;
 if(k===' '){e.preventDefault();w('RC');tt('Recorded')}
 else if(k==='p'){w('PY');tt('Play')}
 else if(k==='o'){w('ST');tt('Stop')}
 else if(k==='c'){w('CY');tt('Cycle')}
 else if(k==='h'){w('PR:home');tt('Home')}
});
addEventListener('keyup',e=>{keys[e.key.toLowerCase()]=false});

// ── Browser Gamepad API
let lastGP=null;
function pollGP(){
 const gps=navigator.getGamepads?navigator.getGamepads():[];
 let gp=null;for(const g of gps)if(g){gp=g;break}
 document.getElementById('gpd').style.display=gp?'inline-block':'none';
 if(gp){
  sticks[0].gpx=Math.abs(gp.axes[0])>.12?gp.axes[0]:0;
  sticks[0].gpy=Math.abs(gp.axes[1])>.12?gp.axes[1]:0;
  sticks[1].gpx=Math.abs(gp.axes[2])>.12?gp.axes[2]:0;
  sticks[1].gpy=Math.abs(gp.axes[3])>.12?gp.axes[3]:0;
  // Triggers from L2/R2 (buttons 6,7) → wrist roll, gripper
  trigs[0].gp=(gp.buttons[7]?.value||0)-(gp.buttons[6]?.value||0);
  trigs[1].gp=(gp.buttons[5]?.value||0)-(gp.buttons[4]?.value||0);
  // Face buttons: A=rec(0), B=stop(1), X=play(2), Y=cycle(3)
  if(gp.buttons[0]?.pressed&&!lastGP?.[0]){w('RC');tt('Rec')}
  if(gp.buttons[1]?.pressed&&!lastGP?.[1]){w('ST');tt('Stop')}
  if(gp.buttons[2]?.pressed&&!lastGP?.[2]){w('PY');tt('Play')}
  if(gp.buttons[3]?.pressed&&!lastGP?.[3]){w('CY');tt('Cycle')}
  lastGP=gp.buttons.map(b=>b.pressed);
 }
}

// ── Tick: compose keyboard + gamepad + drag, send only on change
function tick(){
 pollGP();
 sticks[0].kbx=(keys.d?1:0)-(keys.a?1:0);
 sticks[0].kby=(keys.s?1:0)-(keys.w?1:0);
 sticks[1].kbx=(keys.l?1:0)-(keys.j?1:0);
 sticks[1].kby=(keys.k?1:0)-(keys.i?1:0);
 trigs[0].kb=(keys.e?1:0)-(keys.q?1:0);
 trigs[1].kb=(keys.x?1:0)-(keys.z?1:0);
 sticks.forEach(s=>{
  s.compose();
  if(s.vx!==s.lvx||s.vy!==s.lvy){s.send();s.lvx=s.vx;s.lvy=s.vy}
 });
 trigs.forEach(t=>{
  if(!t.dragging)t.dy=undefined;
  t.pos();
  if(t.v!==t.lv){t.send();t.lv=t.v}
 });
}
setInterval(tick,50);

function dc(){if(confirm('Clear all recorded poses?')){w('CL');tt('Cleared')}}
cn();
</script></body></html>)raw";

// ─── Serial ─────────────────────────────────────────────────────────────────
void processSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd.startsWith("S ")) {
        int s1 = cmd.indexOf(' '), s2 = cmd.indexOf(' ', s1+1);
        if (s2 < 0) { Serial.println("Usage: S <joint 0-5> <angle>"); return; }
        int i = cmd.substring(s1+1, s2).toInt();
        int a = cmd.substring(s2+1).toInt();
        if (i < 0 || i >= 6) { Serial.println("Joint 0-5"); return; }
        setServo(i, a);
        Serial.printf("[%s] -> %d deg\n", joints[i].name, joints[i].cur);
        pendingBroadcast = true;
    }
    else if (cmd.startsWith("INVERT ")) {
        int j = cmd.substring(7).toInt();
        if (j >= 0 && j < 6) {
            joints[j].invert = !joints[j].invert;
            Serial.printf("[%s] invert = %s\n", joints[j].name, joints[j].invert ? "ON" : "OFF");
        }
    }
    else if (cmd == "TEST") {
        Serial.println("-- Servo channel test --");
        for (int i = 0; i < 6; i++) {
            Serial.printf("  %d %s ch%d  lo=%d hi=%d\n",
                i, joints[i].name, joints[i].ch, joints[i].lo, joints[i].hi);
            setServo(i, joints[i].lo);   delay(600);
            setServo(i, joints[i].home); delay(600);
            setServo(i, joints[i].hi);   delay(600);
            setServo(i, joints[i].home); delay(600);
        }
        pendingBroadcast = true;
    }
    else if (cmd == "HOME")  { applyPreset(POSE_HOME);  pendingBroadcast = true; }
    else if (cmd == "READY") { applyPreset(POSE_READY); pendingBroadcast = true; }
    else if (cmd == "PICK")  { applyPreset(POSE_PICK);  pendingBroadcast = true; }
    else if (cmd == "PLACE") { applyPreset(POSE_PLACE); pendingBroadcast = true; }
    else if (cmd == "REC")   recordPose();
    else if (cmd == "PLAY")  startPlayback();
    else if (cmd == "STOP")  stopPlayback();
    else if (cmd == "CYCLE") isCycling ? stopCycle() : startCycle();
    else if (cmd == "CLEAR") clearRecording();
    else if (cmd == "SAVE")  saveToFlash();
    else if (cmd == "LOAD")  loadFromFlash();
    else if (cmd == "STATUS") {
        Serial.println("-- Joints --");
        for (int i = 0; i < 6; i++)
            Serial.printf("  %d. %-14s ch%-2d  %3d deg  invert=%s\n",
                i, joints[i].name, joints[i].ch, joints[i].cur, joints[i].invert?"Y":"N");
        Serial.printf("  HW joy pair : %s\n", PAIR_NAME[joyMode]);
        Serial.printf("  Recording   : %d / %d\n", seqLen, MAX_POSES);
        Serial.printf("  Playing/Cyc : %s / %s\n", isPlaying?"yes":"no", isCycling?"yes":"no");
        Serial.printf("  WiFi RSSI   : %d dBm\n", (int)WiFi.RSSI());
    }
    else if (cmd == "HELP") {
        Serial.println("S <j> <a> | INVERT <j> | TEST | HOME/READY/PICK/PLACE");
        Serial.println("REC | PLAY | STOP | CYCLE | CLEAR | SAVE | LOAD | STATUS");
    }
    else if (cmd.length() > 0) Serial.println("Unknown. HELP.");
}

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RoboArm 6-DOF (main_new) ===");

    for (int a = 0; a <= 180; a++) {
        long us = SERVO_MIN_US + (a * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
        pwmTable[a] = (uint16_t)((us * 4096UL) / 20000UL);
    }

    servoMutex = xSemaphoreCreateMutex();
    wsQueue    = xQueueCreate(32, sizeof(WsMsg));

    Wire.begin(I2C_SDA, I2C_SCL);
    pca9685.begin();
    pca9685.setOscillatorFrequency(OSC_FREQ);
    pca9685.setPWMFreq(PWM_FREQ_HZ);
    delay(10);
    Serial.println("[PCA9685] OK");

    pinMode(JOY_SW_PIN,    INPUT_PULLUP);
    pinMode(BTN_REC_PIN,   INPUT_PULLUP);
    pinMode(BTN_PLAY_PIN,  INPUT_PULLUP);
    pinMode(BTN_CLR_PIN,   INPUT_PULLUP);
    pinMode(BTN_CYCLE_PIN, INPUT_PULLUP);
    for (int i = 0; i < 6; i++) joyF[i] = (float)joints[i].home;

    calibrateJoystick();
    moveToHome();
    Serial.println("[Servos] Home");

    loadFromFlash();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Joining \"%s\"", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
        delay(400); Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("[WiFi] http://%s\n", WiFi.localIP().toString().c_str());
    else
        Serial.println("[WiFi] FAILED");

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200, "text/html", HTML_PAGE);
    });
    server.begin();
    Serial.println("Send HELP for serial commands.");
}

// ─── Loop ───────────────────────────────────────────────────────────────────
void loop() {
    WsMsg m;
    while (xQueueReceive(wsQueue, &m, 0)) processWsCmd(m.buf);

    processJoystick();
    processWebJog();
    processButtons();
    processPlayback();
    processSerial();

    ws.cleanupClients();

    if (pendingBroadcast || ((joyActive || webJogActive) && millis() - lastBroadMs >= 50)) {
        broadcastStatus();
        lastBroadMs      = millis();
        pendingBroadcast = false;
    }
}