#pragma once

#include <Arduino.h>

// ================================ sensor_config =======================================
#define TRIGGER_PIN 13         // GPIO pin for trigger
#define ECHO_PIN 12            // GPIO pin for echo
#define TIME_OUT 50000         // Timeout for ultrasonic sensor (microseconds)
#define MAX_DISTANCE 400       // Maximum distance (cm)
#define MIN_DISTANCE 2         // Minimum distance (cm)
#define SPEED_OF_SOUND 0.0345  // Speed of sound in cm/us (22C)
#define DISTANCE_THRESHOLD 200 // Distance threshold for obstacle detection (cm)

// ================================ wheelEncoders_config =======================================
#define ENCODER_FL_PIN 17
#define ENCODER_FR_PIN 16
#define ENCODER_BL_PIN 15
#define ENCODER_BR_PIN 14

#define WHEEL_DIAMETER 0.026
#define WHEEL_CIRCUMFERENCE 0.08168
#define TICKS_PER_REV 20
#define DISTANCE_PER_TICK 0.004084
#define WHEEL_BASE 0.205
// ================================ IMU_config =======================================
#define SDA_PIN 11
#define SCL_PIN 10

//  MOTOR DRIVERS (2x L298N)
// ============================================================================

// L298N FRONT MOTORS (back_left & Front-left)

#define PIN_MOTOR_FL_IN1 36
#define PIN_MOTOR_FL_IN2 35

#define PIN_MOTOR_FL__BL_ENA 37

#define PIN_MOTOR_BL_IN3 34
#define PIN_MOTOR_BL_IN4 33

// L298N  BACK MOTORS (Back-right & front_right)

#define PIN_MOTOR_BR_IN1 30
#define PIN_MOTOR_BR_IN2 28
#define PIN_MOTOR_BR_FR_ENB 31

#define PIN_MOTOR_FR_IN3 27
#define PIN_MOTOR_FR_IN4 26

// ============================================================================
// 2. SERVO MOTORS (2x MG995 180°)
// ============================================================================
#define PIN_SERVO_1 5 // Servo 1 Signal (MG995)
#define PIN_SERVO_2 8 // Servo 2 Signal (MG995)

// ================================ communication_config =======================================

// Keep real network credentials out of source control.
inline constexpr char WIFI_SSID[] = "44";
inline constexpr char WIFI_PASSWORD[] = "rabe3a25";

inline constexpr uint16_t UDP_COMMAND_PORT = 8888;
inline constexpr uint16_t UDP_TELEMETRY_PORT = 8889;

inline constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 2000;
inline constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
inline constexpr uint32_t TELEMETRY_PERIOD_MS = 100;