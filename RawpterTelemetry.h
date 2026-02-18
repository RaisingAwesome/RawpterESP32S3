#pragma once

struct RawpterTelemetry
{
  //Telemetry
  int16_t highestThrottlePWM = 1500;
  int16_t lowestThrottlePWM = 2000;
  bool flying = false;
};
