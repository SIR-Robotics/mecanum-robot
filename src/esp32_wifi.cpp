#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>

const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";
const char* ARM_API_BASE = "http://10.4.0.99";

HardwareSerial unoSerial(2);

const uint8_t UNO_RX_PIN = 16;
const uint8_t UNO_TX_PIN = 17;
const uint32_t UNO_BAUD = 9600;

void report(const char* message) {
  Serial.println(message);
  unoSerial.println(message);
}

bool runArmApi(const char* path) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = String(ARM_API_BASE) + path;
  http.begin(url);
  int code = http.POST("");
  http.end();

  return code >= 200 && code < 300;
}

bool fetchRed() {
  return runArmApi("/api/run/red");
}

bool fetchBlue() {
  return runArmApi("/api/run/blue");
}

bool fetchYellow() {
  return runArmApi("/api/run/yellow");
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

void setup() {
  Serial.begin(115200);
  unoSerial.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  connectWifi();
}

void loop() {
  static unsigned long lastReportMs = 0;

  readUnoCommands();

  if (WiFi.status() != WL_CONNECTED) {
    report("WIFI_DISCONNECTED");
    connectWifi();
    lastReportMs = millis();
  }

  if (millis() - lastReportMs >= 5000) {
    lastReportMs = millis();
    String message = "WIFI_CONNECTED " + WiFi.localIP().toString();
    report(message.c_str());
  }
}
