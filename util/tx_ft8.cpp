#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tx_ft8.h"

#include "pico/stdlib.h"

#include "../peripheral_util/pico_si5351/si5351.h"

#include "../common.h"

void tune(int RF_freq) {
  // Just transmit a single tone for testing
}

void send_ft8(uint8_t tones[], uint32_t RF_freq, uint16_t AF_freq) {

  absolute_time_t tone_wait_time;

  // Important
  pre_transmit();
  gpio_put(PTT_PIN, 1);
  gpio_put(LED_PIN, 1);
  si_output_enable(SI5351_CLK1, 1);
  si_output_enable(SI5351_CLK2, 1); // Weird!

  uint32_t tx_freq;
  for (uint8_t i = 0; i < 79; i++) {
    tx_freq = 4 * (RF_freq + AF_freq + tones[i] * 6.25);
    // printf("%d ", tones[i]);
    SI_SETFREQ(1, tx_freq); // Note: CLK 1 (VFO 1) is used for output
    si_evaluate();
    sleep_ms(159);
  }

  si_output_enable(SI5351_CLK1, 0);
  si_output_enable(SI5351_CLK2, 0); // It is weird that I have to do this!
  gpio_put(PTT_PIN, 0);
  gpio_put(LED_PIN, 0);
  sleep_ms(100); // important
  // Turn off the relays -> Move T/R switch to RX (NC) position. RX Mode.
  gpio_put(relay_1, 1);
  gpio_put(relay_2, 1);
  sleep_ms(50);
}
