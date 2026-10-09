#include <Arduino.h>
#include <sensors.h>
#include "config.h"
#include "robot_types.h"
#include <Wire.h>
#include <MPU6050.h>

// put function declarations here:

void setup()
{
  Serial.begin(115200); // ESP32 Serial Monitor
  initSensors();        // Initialize sensors
  initEncoders();       // Initialize wheel encoders
  initIMU();            // Initialize IMU
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
}
