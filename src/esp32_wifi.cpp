// To flash this code, run:
// pio run -e esp32 -t upload --upload-port /dev/cu.usbserial-0001

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";

const char* MQTT_HOST = "mqtt.favoriot.com";
const uint16_t MQTT_PORT = 1883;

HardwareSerial unoSerial(2);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

const uint8_t UNO_RX_PIN = 16;
const uint8_t UNO_TX_PIN = 17;
const uint32_t UNO_BAUD = 9600;

uint32_t nextArmRequestId = 1;
uint32_t lastArmResultId = 0;
bool lastArmResultOk = false;
String queuedColor;
uint32_t activeChallengeId = 0;
uint32_t forwardedCommandId = 0;
long forwardedCommandKey = -1;

void connectFavoriot();

String rpcTopic() {
  return String(DEVICE_ACCESS_TOKEN) + "/v2/rpc";
}

String streamTopic() {
  return String(DEVICE_ACCESS_TOKEN) + "/v2/streams";
}

bool favoriotConfigured() {
  return DEVICE_ACCESS_TOKEN[0] != '\0';
}

void publishFavoriot(const char* key, const char* value) {
  if (!mqtt.connected()) return;

  String escaped(value);
  escaped.replace("\\", "\\\\");
  escaped.replace("\"", "\\\"");
  escaped.replace("\r", " ");
  escaped.replace("\n", " ");

  String payload = "{\"device_developer_id\":\"" + String(DEVICE_DEVELOPER_ID) +
                   "\",\"data\":{\"" + key + "\":\"" + escaped + "\"}}";
  mqtt.publish(streamTopic().c_str(), payload.c_str());
}

void actionLog(const char* message) {
  publishFavoriot("action", message);
}

void publishFrontend(const char* type, uint32_t id, long key = -1) {
  if (!mqtt.connected()) return;
  char payload[96];
  if (key >= 0) {
    snprintf(payload, sizeof(payload),
             "{\"to\":\"frontend\",\"type\":\"%s\",\"id\":%lu,\"key\":%ld}",
             type, (unsigned long)id, key);
  } else {
    snprintf(payload, sizeof(payload),
             "{\"to\":\"frontend\",\"type\":\"%s\",\"id\":%lu}",
             type, (unsigned long)id);
  }
  mqtt.publish(rpcTopic().c_str(), payload);
}

void report(const char* message) {
  Serial.println(message);
  unoSerial.println(message);
  publishFavoriot("status", message);
}

long readJsonLong(const String& message, const char* key) {
  int keyPos = message.indexOf(key);
  if (keyPos < 0) return -1;

  int colon = message.indexOf(':', keyPos);
  if (colon < 0) return -1;

  return message.substring(colon + 1).toInt();
}

bool publishArmCommand(const char* color, uint32_t id) {
  if (!mqtt.connected()) return false;

  char payload[80];
  snprintf(payload, sizeof(payload), "{\"to\":\"arm\",\"command\":\"%s\",\"id\":%lu}",
           color, (unsigned long)id);
  return mqtt.publish(rpcTopic().c_str(), payload);
}

bool waitForArmResult(uint32_t id, uint32_t timeoutMs = 15000) {
  unsigned long startMs = millis();
  while (millis() - startMs < timeoutMs) {
    connectFavoriot();
    mqtt.loop();
    if (lastArmResultId == id) return lastArmResultOk;
    delay(10);
  }
  return false;
}

bool runColor(const char* color) {
  connectFavoriot();
  uint32_t id = nextArmRequestId++;
  lastArmResultId = 0;
  bool ok = publishArmCommand(color, id) && waitForArmResult(id);

  char result[24];
  snprintf(result, sizeof(result), "%s_%s", color, ok ? "ok" : "fail");
  publishFavoriot("arm", result);
  return ok;
}

long readCommandKey(const String& message) {
  String value = message;
  value.trim();
  if (value.startsWith("ir_key")) value = value.substring(6);

  bool numeric = value.length() > 0;
  for (unsigned int i = 0; i < value.length(); i++) {
    if (!isDigit(value[i]) && !isSpace(value[i])) numeric = false;
  }
  if (numeric) return value.toInt();

  const char* keys[] = {"\"ir\"", "\"key\"", "\"case\"", "\"command\"", "\"value\""};
  for (unsigned int i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
    int keyPos = message.indexOf(keys[i]);
    if (keyPos < 0) continue;

    int colon = message.indexOf(':', keyPos);
    if (colon < 0) continue;

    int start = colon + 1;
    while (start < (int)message.length() && !isDigit(message[start]) && message[start] != '-') start++;
    if (start < (int)message.length()) return message.substring(start).toInt();
  }
  return -1;
}

bool fetchRed() {
  return runColor("red");
}

bool fetchBlue() {
  return runColor("blue");
}

bool fetchYellow() {
  return runColor("yellow");
}

void handleUnoCommand(const String& command) {
  if (command == "RUN_RED") {
    report(fetchRed() ? "ARM_RED_OK" : "ARM_RED_FAIL");
  } else if (command == "RUN_BLUE") {
    report(fetchBlue() ? "ARM_BLUE_OK" : "ARM_BLUE_FAIL");
  } else if (command == "RUN_YELLOW") {
    report(fetchYellow() ? "ARM_YELLOW_OK" : "ARM_YELLOW_FAIL");
  } else if (command.startsWith("ACTION_LOG ")) {
    String message = command.substring(11);
    actionLog(message.c_str());
    if (message.startsWith("favoriot key ")) {
      long key = message.substring(14).toInt();
      if (key == forwardedCommandKey) publishFrontend("ack", forwardedCommandId, key);
    } else if (message == "Done." && activeChallengeId != 0) {
      publishFrontend("completed", activeChallengeId);
      activeChallengeId = 0;
    }
  }
}

void handleFavoriotMessage(char* topic, byte* payload, unsigned int length) {
  String message((const char*)payload, length);
  message.toLowerCase();
  Serial.printf("Favoriot %s: %s\n", topic, message.c_str());

  if (message.indexOf("\"to\":\"mecanum\"") >= 0 &&
      (message.indexOf("_ok") >= 0 || message.indexOf("_fail") >= 0)) {
    long id = readJsonLong(message, "\"id\"");
    if (id >= 0) {
      lastArmResultId = (uint32_t)id;
      lastArmResultOk = message.indexOf("_ok") >= 0;
    }
    return;
  }

  if (message.indexOf("\"to\":\"arm\"") >= 0) return;
  if (message.indexOf("\"to\":\"frontend\"") >= 0) return;

  long key = readCommandKey(message);
  if (key >= 0) {
    long id = readJsonLong(message, "\"id\"");
    if (id < 0) return;
    forwardedCommandId = (uint32_t)id;
    forwardedCommandKey = key;
    if (key == 13) activeChallengeId = forwardedCommandId;
    unoSerial.print("IR_KEY ");
    unoSerial.print(key);
    unoSerial.print(' ');
    unoSerial.println(id);
    publishFavoriot("command", "sent");
    return;
  }

  if (message.indexOf("red") >= 0) queuedColor = "red";
  else if (message.indexOf("blue") >= 0) queuedColor = "blue";
  else if (message.indexOf("yellow") >= 0) queuedColor = "yellow";
}

void readUnoCommands() {
  static String command;

  while (unoSerial.available()) {
    char c = unoSerial.read();
    if (c == '\n') {
      command.trim();
      if (command.length() > 0) handleUnoCommand(command);
      command = "";
    } else if (c != '\r') {
      command += c;
    }
  }
}

void connectWifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    String message = "WIFI_CONNECTED " + WiFi.localIP().toString();
    report(message.c_str());
  } else {
    report("WIFI_FAILED");
  }
}

void connectFavoriot() {
  static unsigned long lastAttemptMs = 0;

  if (!favoriotConfigured()) {
    report("MQTT_NOT_CONFIGURED");
    return;
  }
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (lastAttemptMs != 0 && millis() - lastAttemptMs < 3000) return;
  lastAttemptMs = millis();

  String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  report("MQTT_CONNECTING");
  if (mqtt.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN)) {
    mqtt.subscribe(rpcTopic().c_str());
    report("Device conencted");
    String wifiMessage = "WIFI_CONNECTED " + WiFi.localIP().toString();
    report(wifiMessage.c_str());
  } else {
    char message[24];
    snprintf(message, sizeof(message), "MQTT_FAILED %d", mqtt.state());
    report(message);
  }
}

void setup() {
  Serial.begin(115200);
  unoSerial.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  report("ESP32_FAVORIOT_BOOT");
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(handleFavoriotMessage);
  connectWifi();
  connectFavoriot();
}

void loop() {
  static unsigned long lastReportMs = 0;

  readUnoCommands();

  if (WiFi.status() != WL_CONNECTED) {
    report("WIFI_DISCONNECTED");
    connectWifi();
    lastReportMs = millis();
  }

  connectFavoriot();
  mqtt.loop();

  if (queuedColor.length() > 0) {
    String color = queuedColor;
    queuedColor = "";
    runColor(color.c_str());
  }

  if (millis() - lastReportMs >= 5000) {
    lastReportMs = millis();
    // String message = "WIFI_CONNECTED " + WiFi.localIP().toString();
    // report(message.c_str());

    char mqttMessage[32];
    snprintf(mqttMessage, sizeof(mqttMessage), "MQTT_%s %d",
             mqtt.connected() ? "CONNECTED" : "DISCONNECTED", mqtt.state());
    report(mqttMessage);
  }
}
