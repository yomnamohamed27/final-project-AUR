# ESP32 communication contract

The ESP32 communication task uses dependency-free CSV datagrams over UDP. The
ROS node on the host computer is responsible for publishing commands and
translating telemetry into ROS topics.

## Network

- ESP32 command listener: UDP port `8888`
- ESP32 telemetry sender: UDP port `8889`
- Wi-Fi credentials and limits: [`include/config.h`](../../include/config.h)

The ESP32 learns the telemetry destination from the source address of the last
valid command packet. Therefore the ROS node should send commands from the same
host and keep sending them periodically.

## Command packet

Send this line to port `8888` at least every 200 ms:

```text
CMD,<linear_mps>,<angular_rps>,<servo1_deg>,<servo2_deg>
```

Example:

```text
CMD,0.25,0.00,90,120
```

Linear velocity is limited
to +/-1.5 m/s and angular velocity to +/-6 rad/s. Servo angles are limited to
0-180 degrees.

Valid commands receive:

```text
ACK,<millis>
```

Malformed or out-of-range commands are ignored. If no valid command arrives
for 500 ms, the ESP32 sets both velocity targets to zero.

## Telemetry packet

The ESP32 sends this line to port `8889` every 100 ms:

```text
TELEMETRY,<x_m>,<y_m>,<theta_rad>,<linear_mps>,<battery_percent>,<mode>,<arm_status>
```

These values are not final and will be changed