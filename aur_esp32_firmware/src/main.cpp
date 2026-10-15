#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "robot_types.h"
#include "motor_control.h"
#include "sensors.h"
#include "ros_comms.h" 

// ============================================================================
// Task periods (ms)
// ============================================================================
static const uint32_t CONTROL_PERIOD_MS    = 20; // must match DT = 0.02 in motor_control.cpp
static const uint32_t IMU_PERIOD_MS        = 20; // 50 Hz
static const uint32_t ULTRASONIC_PERIOD_MS = 60; // HC-SR04 needs >= ~60 ms between pings

// ============================================================================
// Task: speed measurement + PID + motors + servos (every 20 ms, exact)
// ============================================================================
static void controlTask(void *parameter) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while(1) {
        // Failsafe: no valid command from ROS 2 for too long -> stop the robot
        if ((uint32_t)(millis() - last_heartbeat_ms) > COMMAND_TIMEOUT_MS) {
            target_linear_velocity  = 0.0;
            target_angular_velocity = 0.0;
        }

        finalSpeedCalculation(); // encoder ticks -> speed of each wheel
        currentVelocity();       // wheel speeds -> linear / angular velocity
        updateMotors();          // kinematics + PID + L298N + servos

        vTaskDelayUntil(&lastWake, period);
    }
}

// ============================================================================
// Task: IMU (MPU6050) at 50 Hz
// ============================================================================
static void imuTask(void *parameter) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_PERIOD_MS);

    while(1) {
        readIMU();
        vTaskDelayUntil(&lastWake, period);
    }
}

// ============================================================================
// Task: Ultrasonic (slow and blocking, so it has its own low-priority task)
// ============================================================================
static void ultrasonicTask(void *parameter) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS);

    while(1) {
        ultrasonic_distance_cm = getDistance();
        vTaskDelayUntil(&lastWake, period);
    }
}

// ============================================================================
// setup
// ============================================================================
void setup() {
    Serial.begin(115200);

    // 1) Hardware init - motors first so everything starts in a safe (stopped) state
    //initMotors();
    //initEncoders();
    //initSensors();
    //initIMU();

    // 2) Communication (WiFi + UDP)
    if (!RosComms::begin()) {
        Serial.println("RosComms::begin() failed");
    }

    // 3) Create tasks
    //    Core 0: WiFi/communication (the WiFi stack already runs there)
    //    Core 1: control loop and sensors
    //xTaskCreatePinnedToCore(controlTask,    "control",    4096, nullptr, 3, nullptr, 1);
    //xTaskCreatePinnedToCore(imuTask,        "imu",        4096, nullptr, 2, nullptr, 1);
    // xTaskCreatePinnedToCore(RosComms::task, "ros_comms",  8192, nullptr, 2, nullptr, 0);
    //xTaskCreatePinnedToCore(ultrasonicTask, "ultrasonic", 3072, nullptr, 1, nullptr, 1);
}

// ============================================================================
// loop: everything runs in tasks, so this just sleeps
// ============================================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}