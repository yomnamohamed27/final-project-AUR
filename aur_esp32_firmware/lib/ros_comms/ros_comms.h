#pragma once

namespace RosComms {

bool begin();
void sendTelemetry();
void task(void* parameter);

}  // namespace RosComms