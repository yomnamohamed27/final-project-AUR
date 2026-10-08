#pragma once

void initMotors();
int calculatePID(float target_speed, float actual_speed, float &integral, float &prev_error);
void setMotorHardware(int in1_pin, int in2_pin, int pwm_pin, float target_speed, int pwm_value) ;
void updateMotors();
