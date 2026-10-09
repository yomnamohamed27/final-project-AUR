#include "ros_comms.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "robot_types.h"

namespace {

WiFiUDP udp;
IPAddress telemetryAddress;
TaskHandle_t communicationTaskHandle = nullptr;
uint32_t lastTelemetryMs = 0;
uint32_t lastWifiAttemptMs = 0;
bool udpReady = false;
bool wifiWasConnected = false;
bool telemetryDestinationKnown = false;

bool parseFloat(const String& value, float& output) {
  char* end = nullptr;
  const float parsed = strtof(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !isfinite(parsed)) {
    return false;
  }
  output = parsed;
  return true;
}

bool parseInt(const String& value, int& output) {
  char* end = nullptr;
  const long parsed = strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' || parsed < INT_MIN ||
      parsed > INT_MAX) {
    return false;
  }
  output = static_cast<int>(parsed);
  return true;
}

void applyCommand(const String& packet) {
  // CMD,<linear_mps>,<angular_rps>,<servo1_deg>,<servo2_deg>
  String fields[5];
  size_t fieldCount = 0;
  int start = 0;
  while (start <= packet.length() && fieldCount < 5) {
    const int separator = packet.indexOf(',', start);
    const int end = separator < 0 ? packet.length() : separator;
    fields[fieldCount++] = packet.substring(start, end);
    if (separator < 0) {
      break;
    }
    start = separator + 1;
  }

  if (fieldCount != 5 || fields[0] != "CMD") {
    return;
  }

  float linear = 0.0f;
  float angular = 0.0f;
  int servo1 = 0;
  int servo2 = 0;
  if (!parseFloat(fields[1], linear) || !parseFloat(fields[2], angular) ||
      !parseInt(fields[3], servo1) || !parseInt(fields[4], servo2) ||
      servo1 < 0 || servo1 > 180 ||
      servo2 < 0 || servo2 > 180 ||
      fabsf(linear) > 1.5f || fabsf(angular) > 6.0f) {
    return;
  }

  target_linear_velocity = linear;
  target_angular_velocity = angular;
  target_servo1_angle = servo1;
  target_servo2_angle = servo2;
  last_heartbeat_ms = millis();
  telemetryAddress = udp.remoteIP();
  telemetryDestinationKnown = true;

  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print("ACK,");
  udp.print(last_heartbeat_ms);
  udp.endPacket();
}

void stopCommandIfTimedOut() {
  if (millis() - last_heartbeat_ms > COMMAND_TIMEOUT_MS) {
    target_linear_velocity = 0.0f;
    target_angular_velocity = 0.0f;
  }
}

}  // namespace

namespace RosComms {

bool begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  last_heartbeat_ms = millis();
  return xTaskCreate(task, "Task_Comm", 8192, nullptr, 4,
                     &communicationTaskHandle) == pdPASS;
}

void sendTelemetry() {
  if (!udpReady || !telemetryDestinationKnown ||
      WiFi.status() != WL_CONNECTED) {
    return;
  }

  // TELEMETRY,values to be agreed on separated by commas
  udp.beginPacket(telemetryAddress, UDP_TELEMETRY_PORT);
  udp.print("TELEMETRY,");
  //udp.print(imu_yaw, 4);
  //udp.print(",");
  //udp.print(target_linear_velocity, 4);
  //udp.print(",");
  udp.endPacket();
}

void maintainNetwork() {
  const uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    if (udpReady) {
      udp.stop();
      udpReady = false;
    }
    telemetryDestinationKnown = false;

    if (wifiWasConnected) {
      Serial.println("Wi-Fi disconnected; retrying.");
      wifiWasConnected = false;
    }

    if (now - lastWifiAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
      lastWifiAttemptMs = now;
      WiFi.reconnect();
    }
    return;
  }

  if (!wifiWasConnected) {
    wifiWasConnected = true;
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
  }

  if (!udpReady) {
    udpReady = udp.begin(UDP_COMMAND_PORT);
    if (!udpReady) {
      Serial.println("UDP startup failed; retrying.");
    }
  }
}

void task(void* parameter) {
  (void)parameter;
  for (;;) {
    maintainNetwork();

    if (udpReady) {
      const int packetSize = udp.parsePacket(); //check for an incoming packet
      if (packetSize > 0) { //if found
        String packet;
        packet.reserve(static_cast<size_t>(packetSize));
        while (udp.available()) {
          packet += static_cast<char>(udp.read());
        } //read the bytes and build a string
        packet.trim(); //remove whitespaces
        applyCommand(packet);
      }
    }

    stopCommandIfTimedOut(); //if nothing is received for half a second, stop the robot
    if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) { //send telemetry approx every 100ms
      lastTelemetryMs = millis();
      sendTelemetry();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace RosComms