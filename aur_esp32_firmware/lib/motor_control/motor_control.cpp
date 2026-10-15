#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"
#include "robot_types.h"
#include "motor_control.h"

const float TRACK_WIDTH = 0.205;
const float DT = 0.02; // updateMotors() must be called every 20 ms

float Kp = 150.0;
float Ki = 15.0;
float Kd = 2.0;

float integral_FL = 0.0, prev_error_FL = 0.0;
float integral_FR = 0.0, prev_error_FR = 0.0;
float integral_BL = 0.0, prev_error_BL = 0.0;
float integral_BR = 0.0, prev_error_BR = 0.0;

Servo servo1;
Servo servo2;

void initMotors() {

    // L298N #1: Front right & back_right
    pinMode(PIN_MOTOR_FR_IN3, OUTPUT);
    pinMode(PIN_MOTOR_FR_IN4, OUTPUT);
    pinMode(PIN_MOTOR_BR_FR_ENB, OUTPUT);

    pinMode(PIN_MOTOR_BR_IN1 , OUTPUT);
    pinMode(PIN_MOTOR_BR_IN2 , OUTPUT);
    

    // L298N #2: back_left & front_left 
    pinMode(PIN_MOTOR_FL_IN1 , OUTPUT);
    pinMode(PIN_MOTOR_FL_IN2, OUTPUT);
    pinMode(PIN_MOTOR_FL__BL_ENA , OUTPUT);

    pinMode(PIN_MOTOR_BL_IN3, OUTPUT);
    pinMode(PIN_MOTOR_BL_IN4, OUTPUT);

    // Safety Initial State
    digitalWrite(PIN_MOTOR_FL_IN1, LOW);
    digitalWrite(PIN_MOTOR_FL_IN2, LOW);
    digitalWrite(PIN_MOTOR_FR_IN3, LOW);
    digitalWrite(PIN_MOTOR_FR_IN4, LOW);

    digitalWrite(PIN_MOTOR_BL_IN3, LOW);
    digitalWrite(PIN_MOTOR_BL_IN4, LOW);
    digitalWrite(PIN_MOTOR_BR_IN1, LOW);
    digitalWrite(PIN_MOTOR_BR_IN2, LOW);

    analogWrite(PIN_MOTOR_BR_FR_ENB, 0);
    analogWrite(PIN_MOTOR_FL__BL_ENA , 0);
   

    target_speed_front_left  = 0.0;
    target_speed_front_right = 0.0;
    target_speed_back_left   = 0.0;
    target_speed_back_right  = 0.0;

    pwm_front_left  = 0;
    pwm_front_right = 0;
    pwm_back_left   = 0;
    pwm_back_right  = 0;

    servo1.attach(PIN_SERVO_1);
    servo2.attach(PIN_SERVO_2);

    int s1 = constrain(target_servo1_angle, 0, 180);
    int s2 = constrain(target_servo2_angle, 0, 180);
    servo1.write(s1);
    servo2.write(s2);

    current_servo1_angle = s1;
    current_servo2_angle = s2;
}

// Reset all PID memory (used on emergency stop / when stopped)
static void resetPID() {
    integral_FL = integral_FR = integral_BL = integral_BR = 0.0;
    prev_error_FL = prev_error_FR = prev_error_BL = prev_error_BR = 0.0;
}

// PID works on speed MAGNITUDE (always >= 0).
// Direction is handled separately by setMotorHardware() from the sign of target_speed.
// The encoders are single-channel (no direction), so measured speed is treated as
// unsigned; this avoids the doubled error when driving backwards.
int calculatePID(float target_speed, float actual_speed, float &integral, float &prev_error) {

    float target_mag = fabsf(target_speed);
    float actual_mag = fabsf(actual_speed);

    // Stopped: clear memory and output zero
    if (target_mag < 0.01f) {
        integral = 0.0;
        prev_error = 0.0;
        return 0;
    }

    float error = target_mag - actual_mag;

    integral += error * DT;
    integral = constrain(integral, -1.0f, 1.0f); // anti-windup

    float derivative = (error - prev_error) / DT;
    prev_error = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Clamp (no abs): if we overshoot, output goes down instead of pushing harder
    return constrain((int)output, 0, 255);
}

void setMotorHardware(int in1_pin, int in2_pin, int pwm_pin, float target_speed, int pwm_value) {
    if (emergency_stop || fabsf(target_speed) < 0.001f) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        analogWrite(pwm_pin, 0);

    } else if (target_speed > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        analogWrite(pwm_pin, pwm_value);
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(pwm_pin, pwm_value);
    }
}

void updateMotors() {

    // Differential drive kinematics
    float v_left  = target_linear_velocity - (target_angular_velocity * (TRACK_WIDTH / 2.0f));
    float v_right = target_linear_velocity + (target_angular_velocity * (TRACK_WIDTH / 2.0f));

    // On emergency stop: zero targets and clear PID so nothing is stored
    if (emergency_stop) {
        v_left  = 0.0;
        v_right = 0.0;
        resetPID();
    }

    target_speed_front_left  = v_left;
    target_speed_back_left   = v_left;
    target_speed_front_right = v_right;
    target_speed_back_right  = v_right;

    // PID Speed Control Loop
    pwm_front_left  = calculatePID(target_speed_front_left,  speed_front_left,  integral_FL, prev_error_FL);
    pwm_front_right = calculatePID(target_speed_front_right, speed_front_right, integral_FR, prev_error_FR);
    pwm_back_left   = calculatePID(target_speed_back_left,   speed_back_left,   integral_BL, prev_error_BL);
    pwm_back_right  = calculatePID(target_speed_back_right,  speed_back_right,  integral_BR, prev_error_BR);

    // Write outputs to L298N motor drivers
    setMotorHardware(PIN_MOTOR_FL_IN1, PIN_MOTOR_FL_IN2, PIN_MOTOR_FL__BL_ENA, target_speed_front_left,  pwm_front_left);
    setMotorHardware(PIN_MOTOR_FR_IN3, PIN_MOTOR_FR_IN4, PIN_MOTOR_BR_FR_ENB, target_speed_front_right, pwm_front_right);
    setMotorHardware(PIN_MOTOR_BL_IN3, PIN_MOTOR_BL_IN4, PIN_MOTOR_FL__BL_ENA, target_speed_back_left,   pwm_back_left);
    setMotorHardware(PIN_MOTOR_BR_IN1, PIN_MOTOR_BR_IN2, PIN_MOTOR_BR_FR_ENB, target_speed_back_right,  pwm_back_right);

    // Update servo positions (MG995), limited to 0-180
    int s1 = constrain(target_servo1_angle, 0, 180);
    int s2 = constrain(target_servo2_angle, 0, 180);

    if (current_servo1_angle != s1) {
        servo1.write(s1);
        current_servo1_angle = s1;
    }

    if (current_servo2_angle != s2) {
        servo2.write(s2);
        current_servo2_angle = s2;
    }
}