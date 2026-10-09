#ifndef sensors_h
#define sensors_h

void initSensors();  // Function to initialize sensors
float getDistance(); // Function to get distance from ultrasonic
int checkDistance(); // Function to check if distance is smaller than threshold
int checkTimeout();  // Function to check if timeout occurred

void initEncoders();              // Function to initialize wheel encoders
void encoderFL_ISR();             // Function to increase encoder ticks
void encoderFR_ISR();             // Function to increase encoder ticks
void encoderBL_ISR();             // Function to increase encoder ticks
void encoderBR_ISR();             // Function to increase encoder ticks
float calculateSpeed();           // Function to calculate speed based on encoder ticks
void finalSpeedCalculation();     // Function to calculate final speed for each wheel
float calculateLinearVelocity();  // Function to calculate linear velocity
float calculateAngularVelocity(); // Function to calculate angular velocity
void currentVelocity();           // Function to calculate current linear and angular velocity

void initIMU(); // Function to initialize IMU
void readIMU(); // Function to read IMU data

#endif