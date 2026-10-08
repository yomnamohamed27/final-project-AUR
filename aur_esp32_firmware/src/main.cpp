#include <Arduino.h>
#include <sensors.h>

// put function declarations here:

#include "ros_comms.h"

void setup()
{
  Serial.begin(115200); // ESP32 Serial Monitor
  initSensors();        // Initialize sensors
  if (!RosComms::begin()) {
    Serial.println("Communication startup failed; motors remain stopped.");
  } // begin comms
}

void loop()
{
  float ultrasonic_distance_cm = getDistance();
  Serial.print("Distance: ");
  Serial.print(ultrasonic_distance_cm);
  Serial.println(" cm");
  delay(1000); // Delay for 1 second
  int obstacleDetected = checkDistance();
  if (obstacleDetected)
  {
    Serial.println("Obstacle detected!");
    // part el motors
  }
  else
  {
    Serial.println("No obstacle.");
    // hykml 3adi
  }
  int timeoutOccurred = checkTimeout();
  if (timeoutOccurred)
  {
    Serial.println("Timeout occurred!");
    // Handle timeout case, for example break
  }
  else
  {
    Serial.println("No timeout.");
    // hykml 3adi
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}
