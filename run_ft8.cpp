#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "ft8/pack.h"
#include "ft8/encode.h"

#include "ft8/decode_ft8.h"
#include "ft8/gen_ft8.h"

#include "util/tx_ft8.h"
#include "util/rx_ft8.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

#include "hardware/adc.h"
#include "hardware/dma.h"

#include "pico/multicore.h"
#include "hardware/irq.h"

#include "common.h"

// Uncomment if overclocking > 290MHz
#include "hardware/vreg.h"

// RTC was too slow. So I am using hardware_timer's time_us_64!
#include "peripheral_util/pico_si5351/si5351.h"

// Created by AA1GD Aug. 25, 2021
// OCTOBER 18, 2021 AT 5:14 PM CDT FIRST ON AIR DECODES WITH THIS
#define MY_CALLSIGN "VU3CER"
#define MY_GRID "MK68"

// GPS and time stuff
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
#include <TimeLib.h>

message_info CurrentStation;

UserSendSelection sendChoices;

message_info message_list[kMax_decoded_messages]; // probably needs to be memset cleared before each decode

int16_t signal_for_processing[num_samples_processed] = {0};

uint32_t handler_max_time = 0;

void core1_irq_handler()
{
  // Copy capture_buf to signal_for_processing array, take fft and save to power
  while (multicore_fifo_rvalid())
  {
    uint32_t handler_start = time_us_32();
    uint16_t idx = multicore_fifo_pop_blocking();

    for (int i = 0; i < nfft; i++) {
      fresh_signal[i] -= DC_BIAS;
    }
    inc_extract_power(fresh_signal);
    uint32_t handler_time = (time_us_32() - handler_start) / 1000;
    if (handler_time > handler_max_time) {
      handler_max_time = handler_time;
    }
    // handler MUST BE under 160 ms.
  }

  multicore_fifo_clear_irq(); // Clear interrupt
}

void core1_runner(void)
{
  // Configure Core 1 Interrupt
  printf("second core running!\n");
  multicore_fifo_clear_irq();
  irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_irq_handler);

  irq_set_enabled(SIO_IRQ_PROC1, true);

  // Infinite While Loop to wait for interrupt
  while (1)
  {
    tight_loop_contents();
  }
}

// Prepare relays for TX
void pre_transmit()
{
  // Move T/R switch to TX (NO) position
  gpio_put(relay_1, 0);
  gpio_put(relay_2, 0);
  sleep_ms(30); // Songle SRD
}

void ptt(int state) {
  if (state == 1) {
    gpio_put(relay_1, 1);
    gpio_put(LED_PIN, 1);
  }
  else {
    gpio_put(relay_1, 0);
    gpio_put(LED_PIN, 0);
  }
}

void sync_time_with_gps_with_timeout()
{
  // digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool newData = false;

  Serial.println("GPS Sync Wait...");
  digitalWrite(LED_BUILTIN, LOW);
  Serial2.begin(9600); // https://github.com/earlephilhower/arduino-pico/blob/master/variants/rpipico/pins_arduino.h#L11-L15

  for (unsigned long start = millis(); millis() - start < 32000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
#ifdef debugGPS
      Serial.write(c);
#endif
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
    if (newData && gps.time.isUpdated() && gps.date.isUpdated() && (gps.location.isValid() && gps.location.age() < 2000)) {
      byte Year = gps.date.year();
      byte Month = gps.date.month();
      byte Day = gps.date.day();
      byte Hour = gps.time.hour();
      byte Minute = gps.time.minute();
      byte Second = gps.time.second();
      setTime(Hour, Minute, Second, Day, Month, Year);
      digitalWrite(LED_BUILTIN, HIGH);
      sleep_ms(1000);
      digitalWrite(LED_BUILTIN, LOW);
      sleep_ms(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      sleep_ms(1000);
      digitalWrite(LED_BUILTIN, LOW);
      sleep_ms(1000);
      Serial.println("GPS Sync Done!");
      return;
    }
  }
  Serial.println("GPS Sync Failed!");
}

int main()
{
  // Overclocking the processor
  // 133 MHz is the default, 250 MHz is safe at 1.1V and for flash
  // If using clock > 290MHz, increase voltage and add flash divider
  // See https://raspberrypi.github.io/pico-sdk-doxygen/vreg_8h.html
  // vreg_set_voltage(VREG_VOLTAGE_DEFAULT); // default: VREG_VOLTAGE_1_10 max:VREG_VOLTAGE_1_30
  vreg_set_voltage(VREG_VOLTAGE_1_30); // default: VREG_VOLTAGE_1_10 max:VREG_VOLTAGE_1_30
  // set_sys_clock_khz(250000, true);
  // set_sys_clock_khz(250000, true);
  set_sys_clock_khz(290400, true);
  setup_default_uart();

  // initialize GPIO pins
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 0);
  gpio_init(PTT_PIN);
  gpio_set_dir(PTT_PIN, GPIO_OUT);
  gpio_put(PTT_PIN, 0);
  gpio_init(relay_1);
  gpio_set_dir(relay_1, GPIO_OUT);
  gpio_put(relay_1, 0);
  gpio_init(relay_2);
  gpio_set_dir(relay_2, GPIO_OUT);
  gpio_put(relay_2, 0);

  // start serial connection
  stdio_init_all();

  // setup the adc
  setup_adc();

  // Initialize the Si5351 Oct. 10, 2021
  uint32_t rx_freq = 14074000; // Standard FT8 frequency on 20m band
  uint32_t tx_freq = 14075500;
  si_init();
  // Ri is set to divide the given frequency by 4 for VFO 1 (CLK 1)
  // This is to allow Si5351 to make the .25Hz increments for FT8
  SI_SETFREQ(0, rx_freq); // Note: CLK 0 is used as input for CD2003 / TA2003 receiver chip
  si_evaluate();

  // Make hanning window for fft work
  make_window();

  sleep_ms(1000);

  // Start //
  char message[32];
  uint8_t tones[256];
  int offset = 0;
  unsigned char rec_byte[2];
  unsigned int msg_index;
  bool message_available = true;
  bool send = false;
  bool justSent = false; // must recieve right after sending
  bool autosend = true;
  bool cq_only = false;

  // Sync time
  sync_time_with_gps_with_timeout();
  sleep_ms(1000);
  sync_time_with_gps_with_timeout();
  sleep_ms(1000);
  sync_time_with_gps_with_timeout();
  sleep_ms(1000);
  sync_time_with_gps_with_timeout();
  sleep_ms(1000);
  sync_time_with_gps_with_timeout();

  // launch second core
  multicore_launch_core1(core1_runner);

  // decode loop
  while (true) {
    if (second() % 15 == 0) { // RX
      printf("RECEIVING FOR 12.8 SECONDS\n\n");
      uint32_t start = time_us_32();
      inc_collect_power();
      uint32_t stop = time_us_32();
      printf("Handler max time: %d\n", handler_max_time);
      printf("Recording time: %d us\n", stop - start);
      uint32_t decode_begin = time_us_32();
      uint8_t num_decoded = decode_ft8(message_list);
      printf("Decoding took this long: %d us\n", time_us_32() - decode_begin);
      decode_begin = time_us_32();
    }
  }

  // MAIN PROGRAM LOOP
  /* while (true)
    {
    // CAT
    for (;;) {
      int cmd = getchar_timeout_us(0);
      if (cmd == PICO_ERROR_TIMEOUT) {
        break;
      } else {
        printf("Got serial input!\n");
        if (cmd == 'm') {
          msg_index = 0;
          while (msg_index < sizeof(message)) {
            char c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT)
              break;
            else {
              message[msg_index++] = c;
              message_available = true;
            }
          }
        }
        if (cmd == 'p') {
          if (message_available)
            printf("%s\n", message);
        }
        if (cmd == 'o')
        {
          msg_index = 0;
          // Offset encoded in two bytes
          while (msg_index < 2)
          {
            char c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT)
              break;
            rec_byte[msg_index] = c;
            msg_index++;
          }
          offset = rec_byte[0] + (rec_byte[1] << 8);
          printf("o");
        }
        if (cmd == 't')
        {
          strcpy(message, "VU3CER VU3FOE MK68");
          generate_ft8(message, tones);
          send_ft8(tones, tx_freq, 0);
        }
        if (cmd == '1')
        {
          ptt(1);
        }
        if (cmd == '0')
        {
          ptt(0);
        }
        if (cmd == 'r') {
          // RX
          printf("RECEIVING FOR 12.8 SECONDS\n\n");
          inc_collect_power();
          printf("Handler max time: %d\n", handler_max_time);
          uint32_t decode_begin = time_us_32();
          uint8_t num_decoded = decode_ft8(message_list);
          printf("Decoding took this long: %d us\n", time_us_32() - decode_begin);
          decode_begin = time_us_32();
        }
      }
    }
    sleep_ms(100);
    } */

  return 0;
}
