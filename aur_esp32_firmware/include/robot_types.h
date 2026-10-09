#pragma once

#include <Arduino.h>

// ============================================================================
// 1. ROS 2 COMMANDS (Ahmed)
// ============================================================================
inline float target_linear_velocity = 0.0;      // Target linear speed (m/s)
inline float target_angular_velocity = 0.0;     // Target angular speed (rad/s)
inline int target_servo1_angle = 90;            // Target Servo 1 angle (0-180 deg)
inline int target_servo2_angle = 90;            // Target Servo 2 angle (0-180 deg)
inline volatile uint32_t last_heartbeat_ms = 0; // Last valid command time (ms)
inline volatile uint8_t robot_mode = 0;         // 0=manual, 1=semi-auto, 2=fully-auto
inline volatile bool arm_pick_requested = false;
inline volatile bool arm_drop_requested = false;
// ============================================================================
// 2. MOTOR & SERVO CONTROL (Ammar)
// ============================================================================
// Calculated wheel speeds from kinematics (m/s)
inline float target_speed_front_left = 0.0;
inline float target_speed_front_right = 0.0;
inline float target_speed_back_left = 0.0;
inline float target_speed_back_right = 0.0;

// Motor PWM outputs (0-255)
inline int pwm_front_left = 0;
inline int pwm_front_right = 0;
inline int pwm_back_left = 0;
inline int pwm_back_right = 0;

// Servos Current Angles (MG995 180 deg)
inline int current_servo1_angle = 90; // Active Servo 1 angle (0-180 deg)
inline int current_servo2_angle = 90; // Active Servo 2 angle (0-180 deg)

// ============================================================================
// 3. SENSORS (Yomna)
// ============================================================================

// Wheel Encoders
inline volatile long encoder_front_left_ticks = 0;  // Encoder ticks FL
inline volatile long encoder_front_right_ticks = 0; // Encoder ticks FR
inline volatile long encoder_back_left_ticks = 0;   // Encoder ticks BL
inline volatile long encoder_back_right_ticks = 0;  // Encoder ticks BR

inline float speed_front_left = 0.0;         // Actual speed FL (m/s)
inline float speed_front_right = 0.0;        // Actual speed FR (m/s)
inline float speed_back_left = 0.0;          // Actual speed BL (m/s)
inline float speed_back_right = 0.0;         // Actual speed BR (m/s)
inline float current_linear_velocity = 0.0;  // Actual linear speed (m/s)
inline float current_angular_velocity = 0.0; // Actual angular speed (rad/s)

// IMU MPU-6050 (I2C)
inline float imu_roll = 0.0;    // Roll angle (rad)
inline float imu_pitch = 0.0;   // Pitch angle (rad)
inline float imu_yaw = 0.0;     // Yaw angle (rad)
inline float imu_gyro_x = 0.0;  // Gryo X speed (rad/s)
inline float imu_gyro_y = 0.0;  // Gryo Y speed (rad/s)
inline float imu_gyro_z = 0.0;  // Gyro Z speed (rad/s)
inline float imu_accel_x = 0.0; // Accel X (m/s^2)
inline float imu_accel_y = 0.0; // Accel Y (m/s^2)
inline float imu_accel_z = 0.0; // Accel Z (m/s^2)

// Ultrasonic Sensor
inline float ultrasonic_distance_cm = 0.0; // Front distance (cm)