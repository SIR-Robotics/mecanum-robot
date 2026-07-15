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
const char* ONLINE_PRESENCE = "{\"to\":\"frontend\",\"type\":\"presence\",\"online\":true}";
const char* OFFLINE_PRESENCE = "{\"to\":\"frontend\",\"type\":\"presence\",\"online\":false}";

uint32_t nextArmRequestId = 1;
uint32_t activeChallengeId = 0;
uint32_t forwardedCommandId = 0;
long forwardedCommandKey = -1;
bool onlineAnnounced = false;

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

String readJsonString(const String& message, const char* key) {
  int keyPos = message.indexOf(key);
  if (keyPos < 0) return "";

  int colon = message.indexOf(':', keyPos);
  int start = message.indexOf('"', colon + 1);
  if (colon < 0 || start < 0) return "";

  int end = message.indexOf('"', start + 1);
  return end < 0 ? "" : message.substring(start + 1, end);
}

bool publishTaggingRequest(uint32_t id) {
  if (!mqtt.connected()) return false;

  char payload[72];
  snprintf(payload, sizeof(payload),
           "{\"to\":\"arm\",\"command\":\"check_tagging\",\"id\":%lu}",
           (unsigned long)id);
  return mqtt.publish(rpcTopic().c_str(), payload);
}

void runTagging() {
  connectFavoriot();
  uint32_t id = nextArmRequestId++;
  if (!publishTaggingRequest(id)) unoSerial.println("TAGGING_FAILED mqtt_unavailable");
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

void handleUnoCommand(const String& command) {
  if (command == "CHECK_TAGGING") {
    runTagging();
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

  if (readJsonString(message, "\"to\"") == "mecanum" &&
      readJsonString(message, "\"type\"") == "tagging_result") {
    String status = readJsonString(message, "\"status\"");
    if (status == "started") {
      String color = readJsonString(message, "\"color\"");
      color.toUpperCase();
      unoSerial.print("TAGGING_STARTED ");
      unoSerial.print(readJsonLong(message, "\"tag\""));
      unoSerial.print(' ');
      unoSerial.println(color);
    } else {
      unoSerial.print("TAGGING_FAILED ");
      unoSerial.println(status);
    }
    return;
  }

  if (message.indexOf("\"to\":\"arm\"") >= 0) return;
  if (message.indexOf("\"to\":\"frontend\"") >= 0) return;

  if (readJsonString(message, "\"to\"") == "mecanum" &&
      readJsonString(message, "\"command\"") == "find_color") {
    String color = readJsonString(message, "\"color\"");
    if (color != "red" && color != "blue" && color != "yellow") {
      publishFavoriot("status", "FIND_COLOR_INVALID");
      return;
    }

    color.toUpperCase();
    unoSerial.print("FIND_COLOR ");
    unoSerial.println(color);
    publishFavoriot("command", "find_color_sent");
    return;
  }

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
  String topic = rpcTopic();
  report("MQTT_CONNECTING");
  if (mqtt.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN,
                   topic.c_str(), 0, false, OFFLINE_PRESENCE)) {
    mqtt.subscribe(topic.c_str());
    mqtt.publish(topic.c_str(), ONLINE_PRESENCE, false);
    if (!onlineAnnounced) {
      publishFavoriot("status", "online");
      onlineAnnounced = true;
    }
    Serial.println("Device connected");
    Serial.print("WIFI_CONNECTED ");
    Serial.println(WiFi.localIP());
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
  readUnoCommands();

  if (WiFi.status() != WL_CONNECTED) {
    report("WIFI_DISCONNECTED");
    connectWifi();
  }

  connectFavoriot();
  mqtt.loop();

}
