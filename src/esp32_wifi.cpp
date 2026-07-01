#include <Arduino.h>
#include <WiFi.h>

const char* WIFI_SSID = "ASEM Training";
const char* WIFI_PASS = "Class@Asem";

HardwareSerial unoSerial(2);

const uint8_t UNO_RX_PIN = 16;
const uint8_t UNO_TX_PIN = 17;
const uint32_t UNO_BAUD = 9600;

void report(const char* message) {
  Serial.println(message);
  unoSerial.println(message);
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
