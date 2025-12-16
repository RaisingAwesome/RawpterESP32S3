#ifndef RMT_PPM_READER_H
#define RMT_PPM_READER_H
#ifndef MAX_CHANNELS
#define MAX_CHANNELS 12
#endif
#include <Arduino.h>
#include "esp32-hal-rmt.h"
#include <atomic>
#include <stdint.h>

class RmtPPMReader {
public:
  RmtPPMReader();

  void begin(uint8_t pin, uint32_t rmtFreqHz, uint16_t pulses, uint16_t pulseDurationThreshold);
  static void readTask(void *args);

  uint32_t getChannel(int ch);

private:
  // Config
  uint8_t  gpio = 21;
  uint16_t threshold = 2100;       // us
  uint16_t pulseCount = 8;         // number of channels expected

  // Double buffer for channels
  std::atomic<uint8_t> activeBuf {0};  // 0 or 1
  uint32_t channelsBuf[2][MAX_CHANNELS];

  // RMT scrape buffer
  rmt_data_t rx_symbols[64];       // tune to expected frame size
  size_t     rx_num_symbols = 9;

  // Helpers
  static void parseRMTDataReceived(rmt_data_t *data, size_t len, RmtPPMReader *me, uint32_t *outBuf);

  // Utility
  inline uint8_t inactiveIndex() const { return 1 - activeBuf.load(std::memory_order_relaxed); }
};

#endif // RMT_PPM_READER_H