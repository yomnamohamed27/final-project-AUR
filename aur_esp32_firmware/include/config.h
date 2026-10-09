#ifndef config_h
#define config_h
#include <stdint.h>
// ================================sensor_config=======================================
#define TRIGGER_PIN 12         // GPIO pin for trigger
#define ECHO_PIN 13            // GPIO pin for echo
#define TIME_OUT 50000         // Timeout for ultrasonic sensor (in microseconds)
#define MAX_DISTANCE 400       // Maximum distance (cm) for ultrasonic sensor
#define MIN_DISTANCE 2         // Minmum distance (cm) for ultrasonic sensor
#define SPEED_OF_SOUND 0.0345  // Speed of sound in cm/us (22C)
#define DISTANCE_THRESHOLD 200 // Distance threshold for obstacle detection (cm)

// ================================wheelEncoders_config=======================================
#define ENCODER_FL_PIN 39           // Front Left
#define ENCODER_FR_PIN 36           // Front Right
#define ENCODER_BL_PIN 34           // Back Left
#define ENCODER_BR_PIN 35           // Back Right
#define WHEEL_DIAMETER 0.026        // Diameter of the wheels (m)
#define WHEEL_CIRCUMFERENCE 0.08168 // Circumference of the wheels (m)
#define TICKS_PER_REV 20            // Number of encoder ticks per wheel revolution
#define DISTANCE_PER_TICK 0.004084  // M/tick (WHEEL_CIRCUMFERENCE / TICKS_PER_REV)
#define WHEEL_BASE 0.15             // l7d ma n test 34an n7sbha belzabt // Distance between front and back wheels (m)

//===============================IMU_config=======================================
#define SDA_PIN 21 // I2C SDA pin for IMU
#define SCL_PIN 22 // I2C SCL pin for IMU

// ================================communication_config=======================================

// Keep real network credentials out of source control.
inline constexpr char WIFI_SSID[] = "AUR_ROBOT_WIFI";
inline constexpr char WIFI_PASSWORD[] = "CHANGE_ME";
inline constexpr uint16_t UDP_COMMAND_PORT = 8888;
inline constexpr uint16_t UDP_TELEMETRY_PORT = 8889;
inline constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 2000;
inline constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
inline constexpr uint32_t TELEMETRY_PERIOD_MS = 100;

#endif