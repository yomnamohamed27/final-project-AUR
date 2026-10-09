#include <Arduino.h>
#include "sensors.h"
#include "config.h"
#include "robot_types.h"
#include <Wire.h>
#include <MPU6050.h>
//------------------------------ultreasonic sensor--------------------------------

// Function to initialize sensor

void initSensors()
{
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

// Function to get distance from ultrasonic sensor
float getDistance()
{
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    long Duration = pulseIn(ECHO_PIN, HIGH, TIME_OUT);
    ultrasonic_distance_cm = Duration * SPEED_OF_SOUND / 2.0;
    return ultrasonic_distance_cm;
}

// check el distance if it is smaller than threshold

int checkDistance()
{
    float distance = getDistance();
    if (distance < DISTANCE_THRESHOLD && distance > MIN_DISTANCE)
    {
        return 1; // Obstacle detected
    }
    else // ay case tania
    {
        return 0; // No obstacle
    }
}

// check if timeout occurred

int checkTimeout()
{
    long Duration = pulseIn(ECHO_PIN, HIGH, TIME_OUT);
    if (Duration == 0)
    {
        return 1; // Timeout occurred
    }
    else
    {
        return 0; // No timeout
    }
}

//------------------------------wheel encoders--------------------------------

// Function to initialize sensor

void initEncoders()
{
    pinMode(ENCODER_FL_PIN, INPUT);
    pinMode(ENCODER_FR_PIN, INPUT);
    pinMode(ENCODER_BL_PIN, INPUT);
    pinMode(ENCODER_BR_PIN, INPUT);
    // attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PIN), encoderFL_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PIN), encoderFR_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_BL_PIN), encoderBL_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_BR_PIN), encoderBR_ISR, RISING);
}

// ISR for encoders

void encoderFL_ISR()
{
    encoder_front_left_ticks++;
}

void encoderFR_ISR()
{
    encoder_front_right_ticks++;
}

void encoderBL_ISR()
{
    encoder_back_left_ticks++;
}

void encoderBR_ISR()
{
    encoder_back_right_ticks++;
}

//------------------------------speed calculation--------------------------------
unsigned long prevTime = 0;
long prevTicks_FL = 0;
long prevTicks_FR = 0;
long prevTicks_BL = 0;
long prevTicks_BR = 0;

float calculateSpeed(long previousTicks, long currentTicks, float deltaTime)
{
    long deltaTicks = currentTicks - previousTicks;
    float distance = deltaTicks * DISTANCE_PER_TICK; // in meters
    float speed = distance / deltaTime;              // in m/s
    return speed;
}

void finalSpeedCalculation()
{
    unsigned long currentTime = millis();

    if (currentTime - prevTime > 100)
    {
        float deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds
        speed_front_left = calculateSpeed(prevTicks_FL, encoder_front_left_ticks, deltaTime);
        speed_front_right = calculateSpeed(prevTicks_FR, encoder_front_right_ticks, deltaTime);
        speed_back_left = calculateSpeed(prevTicks_BL, encoder_back_left_ticks, deltaTime);
        speed_back_right = calculateSpeed(prevTicks_BR, encoder_back_right_ticks, deltaTime);

        // Update previous values for next calculation                         //nfs el fakraa bta3t millis
        prevTicks_FL = encoder_front_left_ticks;
        prevTicks_FR = encoder_front_right_ticks;
        prevTicks_BL = encoder_back_left_ticks;
        prevTicks_BR = encoder_back_right_ticks;
        prevTime = currentTime;
    }
    else
    {
        // Handle the case where deltaTime is zero or negative
        speed_front_left = 0.0;
        speed_front_right = 0.0;
        speed_back_left = 0.0;
        speed_back_right = 0.0;
    }
}

//------------------------------angular/linear velocity--------------------------------

float calculateLinearVelocity()
{
    float leftSpeed = (speed_front_left + speed_back_left) / 2.0;
    float rightSpeed = (speed_front_right + speed_back_right) / 2.0;
    float linearVelocity = (leftSpeed + rightSpeed) / 2.0;
    return linearVelocity;
}

float calculateAngularVelocity()
{
    float leftSpeed = (speed_front_left + speed_back_left) / 2.0;
    float rightSpeed = (speed_front_right + speed_back_right) / 2.0;
    float angularVelocity = (rightSpeed - leftSpeed) / WHEEL_BASE; // el distnace maben el 2 wheels rad/s
    return angularVelocity;
}

void currentVelocity()
{
    current_linear_velocity = calculateLinearVelocity();
    current_angular_velocity = calculateAngularVelocity();
}

//------------------------------IMU--------------------------------
MPU6050 imu;
void initIMU()
{
    Wire.begin(SDA_PIN, SCL_PIN); // I2C pins
    imu.initialize();
    imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // range + or - 2
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // range + or - 250
    if (!imu.testConnection())
    {
        Serial.println("IMU connection failed!");
    }
}

float filtered_accel_x = 0.0;
float filtered_accel_y = 0.0;
float filtered_accel_z = 0.0;

float filtered_gyro_x = 0.0;
float filtered_gyro_y = 0.0;
float filtered_gyro_z = 0.0;

float alpha = 0.2;
bool first_reading = true; // Flag to indicate the first reading
// Filtered = 0.2 × NewReading + 0.8 × PreviousFiltered

unsigned long last_time = 0;
float alpha_angle = 0.98;

void readIMU()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float accX = (float)ax / 16384.0 * 9.81;       // Convert to m/s^2
    float accY = (float)ay / 16384.0 * 9.81;       // Convert to m/s^2
    float accZ = (float)az / 16384.0 * 9.81;       // Convert to m/s^2
    float gyrX = ((float)gx / 131.0) * PI / 180.0; // Convert to rad/s
    float gyrY = ((float)gy / 131.0) * PI / 180.0; // Convert to rad/s
    float gyrZ = ((float)gz / 131.0) * PI / 180.0; // Convert to rad/s

    if (first_reading)
    {
        filtered_accel_x = accX;
        filtered_accel_y = accY;
        filtered_accel_z = accZ;
        filtered_gyro_x = gyrX;
        filtered_gyro_y = gyrY;
        filtered_gyro_z = gyrZ;
        first_reading = false;
    }
    else
    {
        filtered_accel_x = alpha * accX + (1.0 - alpha) * filtered_accel_x;
        filtered_accel_y = alpha * accY + (1.0 - alpha) * filtered_accel_y;
        filtered_accel_z = alpha * accZ + (1.0 - alpha) * filtered_accel_z;
        filtered_gyro_x = alpha * gyrX + (1.0 - alpha) * filtered_gyro_x;
        filtered_gyro_y = alpha * gyrY + (1.0 - alpha) * filtered_gyro_y;
        filtered_gyro_z = alpha * gyrZ + (1.0 - alpha) * filtered_gyro_z;
    }
    imu_accel_x = filtered_accel_x;
    imu_accel_y = filtered_accel_y;
    imu_accel_z = filtered_accel_z;
    imu_gyro_x = filtered_gyro_x;
    imu_gyro_y = filtered_gyro_y;
    imu_gyro_z = filtered_gyro_z;

    unsigned long current_time = micros(); // Calculate time difference

    if (last_time != 0)
    {
        float diffTime = (current_time - last_time) / 1000000.0;

        if (diffTime > 0.0 && diffTime < 0.1)
        {
            // Calculate Roll and Pitch from Accelerometer
            float accel_roll = atan2(filtered_accel_y, filtered_accel_z);

            float accel_pitch = atan2(-1 * filtered_accel_x, sqrt(filtered_accel_y * filtered_accel_y + filtered_accel_z * filtered_accel_z));

            // Complementary Filter for Roll
            imu_roll = alpha_angle * (imu_roll + filtered_gyro_x * diffTime) + (1.0 - alpha_angle) * accel_roll;

            // Complementary Filter for Pitch
            imu_pitch = alpha_angle * (imu_pitch + filtered_gyro_y * diffTime) + (1.0 - alpha_angle) * accel_pitch;

            // Calculate Yaw from Gyroscope
            imu_yaw += filtered_gyro_z * diffTime;
        }
    }

    last_time = current_time;
}