/*
 * Copyright (c) 2010, Loughborough University - Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *         Example to demonstrate-test cc2530 sensor functionality
 *
 *         B1 turns LED_GREEN on and off.
 *
 *         The node takes readings from the various sensors every x seconds and
 *         prints out the results.
 *
 *         We use floats here to translate the AD conversion results to
 *         meaningful values. However, our printf does not have %f support so
 *         we use an ugly hack to print out the value by extracting the integral
 *         part and then the fractional part. Don't try this at home.
 *
 *         Temperature:
 *           Math is correct, the sensor needs calibration per device.
 *           I currently use default values for the math which may result in
 *           very incorrect values in degrees C.
 *           See TI Design Note DN102 about the offset calibration.
 *
 *         Supply Voltage (VDD):
 *           For VDD, math is correct, conversion is correct.
 *           See DN101 for details.
 *
 *         Make sure you enable/disable things in contiki-conf.h
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "contiki.h"
#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/onewire.h"

#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"

#define DEBUG 1
#define ONEWIRE_ON 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
/* We overwrite (read as annihilate) all output functions here */
#define PRINTF(...)
#endif /* DEBUG */
/*---------------------------------------------------------------------------*/
PROCESS(onewire_test_process, "Onewire Test Process");
AUTOSTART_PROCESSES(&onewire_test_process);
/*---------------------------------------------------------------------------*/
static void convert_temp_str(int16_t temp_2c, char *temp_s);
static float convert_temp_float(int16_t temp_2c);
static void temp_conversion_tests();
static void print_rom_id(struct onewire_dev *ow);
static void print_scratch_pad(const char *buf, uint8_t count);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(onewire_test_process, ev, data)
{
  static struct etimer et;

  static struct onewire_dev ow;
  static struct search_rom srom;
  uint8_t buf[10];
  int16_t temp = 0;
  char temp_str[10];
  int neg = 0;
  float sane = 0;
  int dec;
  float frac;

  PROCESS_BEGIN();

  PRINTF("=========================\n");
  PRINTF("Starting Onewire Example.\n");
  PRINTF("=========================\n");

  PRINTF("Temp conversion tests:\n");
  temp_conversion_tests();

  PRINTF("===============\n");
  PRINTF("TESTING ONEWIRE\n");
  PRINTF("===============\n");

#if SINGLE_DROP
  /* read rom address */
  if (onewire_read_rom(&ow)) {
    print_rom_id(&ow);
  } else {
    PRINTF("ERROR: Failed to read ROM id\n");
    continue;
  }
#else
  /* prepare for rom search */
  onewire_search_rom_setup(&srom, &ow);
  PRINTF("Performing ROM search:\n");
  if (onewire_search_rom(&srom, &ow)) {
    PRINTF("ROM search sucessful\n");
    print_rom_id(&ow);
  } else {
    PRINTF("ERROR: ROM search failed\n");
  }
#endif

  /* Set an etimer. We take sensor readings when it expires and reset it. */
  etimer_set(&et, CLOCK_SECOND * 2);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    PRINTF("===============\n");
    PRINTF("Verifying device is present:\n");
    if (onewire_search_rom_verify(&srom, &ow)) {
      PRINTF("Device present\n");
    } else {
      PRINTF("ERROR: Device missing\n");
    }

    /* Get temperature from commected DS18B20 */
    if (ow.family == 0x28) {
      PRINTF("DS18B20 Temperature sensor detected\n");
      if (onewire_match_rom(&ow)) {
        PRINTF("MATCH: Starting temp conversion\n");
        onewire_write_cmd(0x44);
      } else {
        PRINTF("Failed to match ROM\n");
      }
      if (onewire_match_rom(&ow)) {
        PRINTF("MATCH: Reading scratch pad:\n");
        onewire_read_fn_cmd_bytes(0xBE, buf, 9);
        print_scratch_pad(buf, 9);

        temp = (uint16_t)((uint8_t)buf[0] | ((uint8_t)buf[1] << 8));
        convert_temp_str(temp, temp_str);
        PRINTF("Temp: 0x%X = %s C\n", (int16_t)temp, temp_str);

      } else {
        PRINTF("Failed to match ROM\n");
      }
    }
    etimer_reset(&et);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static void print_rom_id(struct onewire_dev *ow) {
  PRINTF("Family: 0x%X\n", ow->family);
  PRINTF("ID: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", ow->id[0],ow->id[1],
      ow->id[2],ow->id[3], ow->id[4],ow->id[5]);
  PRINTF("CRC: 0x%X\n", ow->crc);
  return;
}
/*---------------------------------------------------------------------------*/
static void print_scratch_pad(const char *buf, uint8_t count) {
  int i;

  PRINTF("Scratch pad:\n");
  for (i = 0; i < count; i++) {
    PRINTF("0x%X ", (uint8_t)buf[i]);
  }
  PRINTF("\n");
  return;
}
/*---------------------------------------------------------------------------*/
static float convert_temp_float(int16_t temp_2c) {

  int neg = 0;
  float temp;

  neg = (temp_2c & (1 << 11)) != 0;
  if (neg) {
    temp_2c = ~temp_2c + 1;
  }
  temp = temp_2c * 0.0625f;

  if (neg) {
    temp = -temp;
  }

  return temp;
}
/*---------------------------------------------------------------------------*/
static void convert_temp_str(int16_t temp_2c, char *temp_s) {
  /* Preferred conversion method since float does not always */
  /* handle negative precision */

  int neg = 0;
  int dec;
  float temp;
  int i = 0;

  neg = (temp_2c & (1 << 11)) != 0;

  if (neg) {
    temp_2c = ~temp_2c + 1;
    temp_s[i++] = '-';
  }
  temp = temp_2c * 0.0625f;

  dec = temp;
  temp = temp - dec;
  temp_s[sprintf(temp_s, "%d.%04u",
      dec, (unsigned int)(temp*10000))] = '\0';

  return;
}
/*---------------------------------------------------------------------------*/
static void temp_conversion_tests() {
  int16_t temp;
  char temp_str[10];

  temp = 0x07d0;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0x0550;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0x0191;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0x00A2;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0x0008;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0x0000;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0xFFF8;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0xFF5E;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0xFE6F;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);
  temp = 0xFC90;
  convert_temp_str(temp, temp_str);
  PRINTF("Temp: 0x%X = %s C\n", temp, temp_str);

  return;
}
