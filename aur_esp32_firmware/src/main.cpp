#include <Arduino.h>
#include <sensors.h>
#include "config.h"
#include "robot_types.h"
#include <Wire.h>
#include <MPU6050.h>
#include "ros_comms.h"

void setup()
{
  Serial.begin(115200);

  initSensors();
  initEncoders();
  initIMU();

  if (!RosComms::begin())
  {
    Serial.println("Communication startup failed; motors remain stopped.");
  }
}

void loop()
{
  float ultrasonic_distance_cm = getDistance();

  Serial.print("Distance: ");
  Serial.print(ultrasonic_distance_cm);
  Serial.println(" cm");

  int obstacleDetected = checkDistance();

  if (obstacleDetected)
  {
    Serial.println("Obstacle detected!");
    // Motor handling can be added here
  }
  else
  {
    Serial.println("No obstacle.");
  }

  int timeoutOccurred = checkTimeout();

  if (timeoutOccurred)
  {
    Serial.println("Timeout occurred!");
  }
  else
  {
    Serial.println("No timeout.");
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}