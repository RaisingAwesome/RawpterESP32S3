#include "RmtPPMReader.h"
// Constructor
RmtPPMReader::RmtPPMReader() {
  // Initialize both buffers to 0 (safe default)
  for (int b = 0; b < 2; ++b) {
    for (int i = 0; i < MAX_CHANNELS; ++i) {
      channelsBuf[b][i] = 0;
    }
  }
}

void RmtPPMReader::begin(uint8_t pin, uint32_t rmtFreqHz, uint16_t pulses, uint16_t pulseDurationThreshold) {
  gpio = pin;
  threshold = pulseDurationThreshold;
  pulseCount = (pulses <= MAX_CHANNELS) ? pulses : MAX_CHANNELS;

  // RMT init (RX mode assumed constant)
  if (!rmtInit(gpio, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_1, /*Hz*/ 1000000)) {
    Serial.println("init receiver failed");
    while (true) vTaskDelay(10);
  }

  // Set sync gap max threshold (empirically: 2100 us is safe for FlySky-style PPM)
  rmtSetRxMaxThreshold(gpio, threshold);

  // Create the read task
  if (xTaskCreate(readTask, "RmtPPMReader", 4096, this, 4, NULL) != pdPASS) {
    Serial.println("Failed to create RmtPPMReader readTask");
  }
  vTaskDelay(200); // Give time to sync with radio;
}

void RmtPPMReader::readTask(void *args) {
  RmtPPMReader *me = (RmtPPMReader *)args;
  bool checker = false;

  #if !EASYCHAIR
  esp_log_level_set("*", ESP_LOG_NONE); // suppress noisy prints if needed
  #endif

  while (true) {
    // Scrape RMT HW buffer. rx_num_symbols is both input (desired) and output (actual).
    checker = rmtRead(me->gpio, me->rx_symbols, &me->rx_num_symbols, /*timeout ms*/ 30);

    // If we didn't get the expected number of pulses (pulses + sync), re-sync.
    if ((int)me->rx_num_symbols != me->pulseCount + 1) {
      // Let the next pass sync to the frame boundary via threshold stop.
      me->rx_num_symbols = me->pulseCount + 1; // reset desired read count
      Serial.println("Syncing RMT to PPM...");
      // Write zeros to inactive buffer and publish, so readers get a safe state.
      uint8_t inact = me->inactiveIndex();
      for (size_t i = 0; i < (size_t)me->pulseCount; i++) {
        me->channelsBuf[inact][i] = 0;
      }
      me->activeBuf.store(inact, std::memory_order_release);
    } else {
      // Parse into inactive buffer, then atomically publish by flipping activeBuf.
      uint8_t inact = me->inactiveIndex();
      if (checker) {
        parseRMTDataReceived(me->rx_symbols, me->rx_num_symbols, me, me->channelsBuf[inact]);
      } else {
        for (size_t i = 0; i < (size_t)me->pulseCount; i++) {
          me->channelsBuf[inact][i] = 0;
        }
      }
      me->activeBuf.store(inact, std::memory_order_release);
    }
    taskYIELD();
  }
}

void RmtPPMReader::parseRMTDataReceived(rmt_data_t *data, size_t len, RmtPPMReader *me, uint32_t *outBuf) {
  // Clear output buffer
  for (size_t i = 0; i < me->pulseCount; i++) {
    outBuf[i] = 0;
  }

  // Find sync gap (a long high), then parse channels after it
  size_t index = (size_t)-1;
  for (size_t i = 0; i < len; i++) {
    if ((data[i].duration0 >= (me->threshold - 50)) || (data[i].duration1 >= (me->threshold - 50))) {
      index = i + 1;
      break;
    }
  }

  if (index >= len || index==-1) index = 0; // If no sync found, start at 0 as a fallback

  uint8_t chan = 0;
  for (size_t i = index; i < len && chan < me->pulseCount; i++) {
    uint32_t highDur = 0;

    if (data[i].level0 == 1 && data[i].duration0 > 0) {
      highDur = data[i].duration0;
    } else if (data[i].level1 == 1 && data[i].duration1 > 0) {
      highDur = data[i].duration1;
    }

    // Add spacer (empirically ~400us) to convert to high-to-high interval
    outBuf[chan++] = highDur + 400;
  }
}

uint32_t RmtPPMReader::getChannel(int ch) {
  if (ch < 0 || ch >= (int)pulseCount) return 0;

  // Lock-free read: snapshot active buffer, then read the value.
  // Acquire ensures we see the latest published frame contents.
  uint8_t idx = activeBuf.load(std::memory_order_acquire);
  return channelsBuf[idx][ch];
}
