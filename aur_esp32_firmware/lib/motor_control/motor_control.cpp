#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"
#include "robot_types.h"
#include "motor_control.h"

const float TRACK_WIDTH = 0.205; 
const float DT = 0.02;         


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
   
    // L298N #1: Front Motors
    pinMode(PIN_MOTOR_FL_IN1, OUTPUT);
    pinMode(PIN_MOTOR_FL_IN2, OUTPUT);
    pinMode(PIN_MOTOR_FL_ENA, OUTPUT);

    pinMode(PIN_MOTOR_FR_IN3, OUTPUT);
    pinMode(PIN_MOTOR_FR_IN4, OUTPUT);
    pinMode(PIN_MOTOR_FR_ENB, OUTPUT);

    // L298N #2: Back Motors
    pinMode(PIN_MOTOR_BL_IN1, OUTPUT);
    pinMode(PIN_MOTOR_BL_IN2, OUTPUT);
    pinMode(PIN_MOTOR_BL_ENA, OUTPUT);

    pinMode(PIN_MOTOR_BR_IN3, OUTPUT);
    pinMode(PIN_MOTOR_BR_IN4, OUTPUT);
    pinMode(PIN_MOTOR_BR_ENB, OUTPUT);

    
    // Safety Initial State
   
    digitalWrite(PIN_MOTOR_FL_IN1, LOW);
    digitalWrite(PIN_MOTOR_FL_IN2, LOW);
    digitalWrite(PIN_MOTOR_FR_IN3, LOW);
    digitalWrite(PIN_MOTOR_FR_IN4, LOW);

    digitalWrite(PIN_MOTOR_BL_IN1, LOW);
    digitalWrite(PIN_MOTOR_BL_IN2, LOW);
    digitalWrite(PIN_MOTOR_BR_IN3, LOW);
    digitalWrite(PIN_MOTOR_BR_IN4, LOW);

    analogWrite(PIN_MOTOR_FL_ENA, 0);
    analogWrite(PIN_MOTOR_FR_ENB, 0);
    analogWrite(PIN_MOTOR_BL_ENA, 0);
    analogWrite(PIN_MOTOR_BR_ENB, 0);

    
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

    
    servo1.write(target_servo1_angle);
    servo2.write(target_servo2_angle);

   
    current_servo1_angle = target_servo1_angle;
    current_servo2_angle = target_servo2_angle;
}



int calculatePID(float target_speed, float actual_speed, float &integral, float &prev_error) {
   
    float error = target_speed - actual_speed;
    
    
    if (abs(target_speed) < 0.01) {
        integral = 0.0;
    } else {
        integral += error * DT;
        integral = constrain(integral, -1.0f, 1.0f);
    }

    float derivative = (error - prev_error) / DT;
    prev_error = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
   
    int pwm_val = (int)abs(output);
    return constrain(pwm_val, 0, 255);
}


void setMotorHardware(int in1_pin, int in2_pin, int pwm_pin, float target_speed, int pwm_value) {
    if (emergency_stop || abs(target_speed) < 0.001) {
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
    
    float v_left  = target_linear_velocity - (target_angular_velocity * (TRACK_WIDTH / 2.0f));
    float v_right = target_linear_velocity + (target_angular_velocity * (TRACK_WIDTH / 2.0f));

    target_speed_front_left  = v_left;
    target_speed_back_left   = v_left;
    target_speed_front_right = v_right;
    target_speed_back_right  = v_right;

    
    //  PID Speed Control Loop

    pwm_front_left  = calculatePID(target_speed_front_left,  speed_front_left,  integral_FL, prev_error_FL);
    pwm_front_right = calculatePID(target_speed_front_right, speed_front_right, integral_FR, prev_error_FR);
    pwm_back_left   = calculatePID(target_speed_back_left,   speed_back_left,   integral_BL, prev_error_BL);
    pwm_back_right  = calculatePID(target_speed_back_right,  speed_back_right,  integral_BR, prev_error_BR);

    
    //  Write Outputs to L298N Motor Drivers
   
    setMotorHardware(PIN_MOTOR_FL_IN1, PIN_MOTOR_FL_IN2, PIN_MOTOR_FL_ENA, target_speed_front_left,  pwm_front_left);
    setMotorHardware(PIN_MOTOR_FR_IN3, PIN_MOTOR_FR_IN4, PIN_MOTOR_FR_ENB, target_speed_front_right, pwm_front_right);
    setMotorHardware(PIN_MOTOR_BL_IN1, PIN_MOTOR_BL_IN2, PIN_MOTOR_BL_ENA, target_speed_back_left,   pwm_back_left);
    setMotorHardware(PIN_MOTOR_BR_IN3, PIN_MOTOR_BR_IN4, PIN_MOTOR_BR_ENB, target_speed_back_right,  pwm_back_right);

    
    //  Update Servos Positions (MG995)
    
    if (current_servo1_angle != target_servo1_angle) {
        servo1.write(target_servo1_angle);
        current_servo1_angle = target_servo1_angle;
    }

    if (current_servo2_angle != target_servo2_angle) {
        servo2.write(target_servo2_angle);
        current_servo2_angle = target_servo2_angle;
    }
}