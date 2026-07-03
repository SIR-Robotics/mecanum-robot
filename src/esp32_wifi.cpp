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

  char payload[180];
  snprintf(payload, sizeof(payload),
           "{\"device_developer_id\":\"%s\",\"data\":{\"%s\":\"%s\"}}",
           DEVICE_DEVELOPER_ID, key, value);
  mqtt.publish(streamTopic().c_str(), payload);
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
  }
}

void handleFavoriotMessage(char* topic, byte* payload, unsigned int length) {
  String message((const char*)payload, length);
  message.toLowerCase();
  Serial.printf("Favoriot %s: %s\n", topic, message.c_str());

  if (message.indexOf("\"to\":\"mecanum\"") >= 0) {
    long id = readJsonLong(message, "\"id\"");
    if (id >= 0) {
      lastArmResultId = (uint32_t)id;
      lastArmResultOk = message.indexOf("_ok") >= 0;
    }
    return;
  }

  if (message.indexOf("\"to\":\"arm\"") >= 0) return;

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

  if (!favoriotConfigured() || WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  if (lastAttemptMs != 0 && millis() - lastAttemptMs < 3000) return;
  lastAttemptMs = millis();

  String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.println("Connecting to Favoriot MQTT...");
  if (mqtt.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN)) {
    mqtt.subscribe(rpcTopic().c_str());
    report("MQTT_CONNECTED");
  } else {
    Serial.printf("Favoriot MQTT failed: %d\n", mqtt.state());
    report("MQTT_FAILED");
  }
}

void setup() {
  Serial.begin(115200);
  unoSerial.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
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
    String message = "WIFI_CONNECTED " + WiFi.localIP().toString();
    report(message.c_str());
  }
}
