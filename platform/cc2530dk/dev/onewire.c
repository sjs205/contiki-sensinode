/*
 * Copyright (c) 2009, University of Colombo School of Computing
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
 *
 */

/*
 * Device driver for a bitbanging onewire bus
 * Heavly based on the Maxim Integrated application notes:
 *  * 126 "1-Wire Communications Through Software".
 *  * 187 "1-Wire Search Algorithm"
 */

#include <string.h>
#include "dev/port.h"
#include "sys/clock.h"
#include "onewire.h"

#define READ_ROM_CMD        0x33
#define READ_SCRATCHPAD_CMD 0xBE
#define SEARCH_ROM_CMD      0xF0
#define MATCH_ROM_CMD       0x55
#define SKIP_ROM_CMD        0xCC
#define CONVERT_CMD         0x44

/* Currently, only single drop detected */
#define SINGLE_DROP        0

/* 1-wire is at Port1.2 */
#define ONEWIRE_PORT 1
#define ONEWIRE_PIN 2
#define ONEWIRE_PIN_DEF P1_2
#define ONEWIRE_PIN_MASK _BV(2)

#define SET_PIN_INPUT() PORT_DIR_INPUT(ONEWIRE_PORT, ONEWIRE_PIN)
#define SET_PIN_OUTPUT() PORT_DIR_OUTPUT(ONEWIRE_PORT, ONEWIRE_PIN)

#define OUTP_1() PORT_SET(ONEWIRE_PORT, ONEWIRE_PIN)
/* The following failes to compile */
/*#define OUTP_0() PORT_CLEAR(ONEWIRE_PORT, ONEWIRE_PIN) */
#define OUTP_0() do { ONEWIRE_PIN_DEF = 0; } while(0)

#define PIN_INIT() do {                  \
                     SET_PIN_INPUT();    \
                     OUTP_0();           \
                   } while(0)


/* Drive the one wire interface low */
#define OW_DRIVE() do { \
                     SET_PIN_OUTPUT(); \
                     OUTP_0();      \
                   } while (0)

/* Release the one wire by turning on the internal pull-up. */
#define OW_RELEASE() do {                  \
                       SET_PIN_INPUT();    \
                       OUTP_1();           \
                     } while (0)

/* Read one bit. */
#define INP() PORT_READ(ONEWIRE_PORT, ONEWIRE_PIN)

/*
 * Delay times in us.
 */
#define tA 6          /* min-5, recommended-6, max-15 */
#define tB 64         /* min-59, recommended-64, max-N/A */
#define tC 60         /* min-60, recommended-60, max-120 */
#define tD 10         /* min-5.3, recommended-10, max-N/A */
#define tE 9          /* min-0.3, recommended-9, max-9.3 */
#define tF 55         /* min-50, recommended-55, max-N/A */
#define tG 0          /* min-0, recommended-0, max-0 */
#define tH 480        /* min-480, recommended-480, max-640 */
#define tI 70         /* min-60.3, recommended-70, max-75.3 */
#define tJ 410        /* min-410, recommended-410, max-N/A */
/*---------------------------------------------------------------------------*/
#define udelay(u) clock_delay_usec(u);
/*---------------------------------------------------------------------------*/
static uint8_t
reset(void)
{
  uint8_t result;
  OW_DRIVE();
  udelay(500);     /* 480 < tH < 640 */
  OW_RELEASE();    /* Releases the bus */
  udelay(tI);
  result = INP();
  udelay(tJ);
  return result;
}
/*---------------------------------------------------------------------------*/
static void
write_bit(uint8_t bit)
{
  if (bit & 0x01) {
    OW_DRIVE();
    udelay(tA);
    OW_RELEASE();    /* Releases the bus */
    udelay(tB);
  } else {
    OW_DRIVE();
    udelay(tC);
    OW_RELEASE();    /* Releases the bus */
    udelay(tD);
  }
  return;
}
/*---------------------------------------------------------------------------*/
static void
write_byte(uint8_t byte)
{
  uint8_t i = 7;
  do {
    if (byte & 0x01) {
      OW_DRIVE();
      udelay(tA);
      OW_RELEASE();    /* Releases the bus */
      udelay(tB);
    } else {
      OW_DRIVE();
      udelay(tC);
      OW_RELEASE();    /* Releases the bus */
      udelay(tD);
    }
    if (i == 0)
      return;
    i--;
    byte >>= 1;
  } while (1);
}
/*---------------------------------------------------------------------------*/
static void
write_bytes(uint8_t *bytes, unsigned count)
{
  int i;

  for (i = 0; i < count; i++) {
    write_byte(bytes[i]);
  }
  return;
}
/*---------------------------------------------------------------------------*/
static unsigned
read_bit(void)
{
  OW_DRIVE();
  udelay(tA);
  OW_RELEASE();	/* Releases the bus */
  udelay(tE);
  if (INP()) {
    udelay(tF);
    return 1;
  } else {
    udelay(tF);
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
static unsigned
read_byte(void)
{
  unsigned result = 0;
  int i = 7;
  do {
    OW_DRIVE();
    udelay(tA);
    OW_RELEASE();	/* Releases the bus */
    udelay(tE);
    if (INP())
      result |= 0x80;	/* LSbit first */
    udelay(tF);
    if (i == 0)
      return result;
    i--;
    result >>= 1;
  } while (1);
}
/*---------------------------------------------------------------------------*/
/* Polynomial ^8 + ^5 + ^4 + 1 */
static unsigned
crc8_add(unsigned acc, unsigned byte)
{
  int i;
  acc ^= byte;
  for (i = 0; i < 8; i++)
    if (acc & 1)
      acc = (acc >> 1) ^ 0x8c;
    else
      acc >>= 1;

  return acc;
}
/*---------------------------------------------------------------------------*/
  int
onewire_init()
{
  PIN_INIT();

  return 1;
}
/*---------------------------------------------------------------------------*/
int
onewire_search_rom(struct search_rom *srom, struct onewire_dev *ow)
{
  uint8_t id_bit_number = 1;
  uint8_t last_zero = 0;
  uint8_t rom_byte_number = 0;
  uint8_t search_result = 0;
  uint8_t id_bit;
  uint8_t cmp_id_bit;
  unsigned char rom_byte_mask = 1;
  unsigned char search_direction;

  unsigned char *rom_no = (unsigned char *)ow;

  DISABLE_INTERRUPTS();

  PIN_INIT();

  /* if the last call was not the last one */
  if (!srom->last_device_flag) {
    if (reset() == 1) {
      /* reset failed, reset search */
      srom->last_discrepancy = 0;
      srom->last_device_flag = false;
      srom->last_family_discrepancy = 0;
      ENABLE_INTERRUPTS();
      return 0;
    }

    /* Search ROM command. */
    DISABLE_INTERRUPTS();
    write_byte(SEARCH_ROM_CMD);  
    ENABLE_INTERRUPTS();

    /* search loop */
    do {
      /* read a bit and its complement */
      DISABLE_INTERRUPTS();
      id_bit = read_bit();
      cmp_id_bit = read_bit();
      ENABLE_INTERRUPTS();

      /* check for no devices on 1-wire */
      if ((id_bit == 1) && (cmp_id_bit == 1)) {
        break;
      } else {
        /* all devices coupled have 0 or 1 */
        if (id_bit != cmp_id_bit) {
          /* bit write value for search */
          search_direction = id_bit;
        } else {
          /* if this discrepancy if before the Last Discrepancy */
          /* on a previous next then pick the same as last time */
          if (id_bit_number < srom->last_discrepancy) {
            search_direction =
              (((unsigned char)ow[rom_byte_number] & rom_byte_mask) > 0);
          } else {
            /* if equal to last pick 1, if not then pick 0 */
            search_direction = (id_bit_number == srom->last_discrepancy);
          }

          /* if 0 was picked then record its position in LastZero */
          if (search_direction == 0)
          {
            last_zero = id_bit_number;
            /* check for Last discrepancy in family */
            if (last_zero < 9) {
              srom->last_family_discrepancy = last_zero;
            }
          }
        }

        /* set or clear the bit in the ROM byte rom_byte_number */
        /* with mask rom_byte_mask */
        if (search_direction == 1) {
           rom_no[rom_byte_number] |= rom_byte_mask;
        } else {
           rom_no[rom_byte_number] &= ~rom_byte_mask;
        }

        /* serial number search direction write bit */
        DISABLE_INTERRUPTS();
        write_bit(search_direction);
        ENABLE_INTERRUPTS();

        /* increment the byte counter id_bit_number */
        /* and shift the mask rom_byte_mask */
        id_bit_number++;
        rom_byte_mask <<= 1;

        /* if the mask is 0 then go to new SerialNum byte */
        /* rom_byte_number and reset mask */
        if (rom_byte_mask == 0) {
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    } while (rom_byte_number < 8);  /* loop until through all ROM bytes 0-7 */

    /* if the search was successful then */
    if (!((id_bit_number < 65) && rom_no[7] != 0)) {
      /* search successful */
      srom->last_discrepancy = last_zero;

      /* check for last device */
      if (srom->last_discrepancy == 0)
        srom->last_device_flag = true;

      search_result = true;
    }
  }

  /* if no device found then reset counters so next 'search' */
  /* will be like a first */
  if (!search_result || !rom_no[0]) {
    srom->last_discrepancy = 0;
    srom->last_device_flag = false;
    srom->last_family_discrepancy = 0;
    search_result = false;
  }

  return search_result;
}
/*---------------------------------------------------------------------------*/
int onewire_search_rom_verify(struct search_rom *srom,
    struct onewire_dev *ow)
{
  unsigned char rom_no[8];
  unsigned char *rom_no_in = (unsigned char *)ow;
  int present = true;
  int i;

  /* backup state */
  uint8_t ld = srom->last_discrepancy;
  uint8_t ldf = srom->last_device_flag;
  uint8_t lfd = srom->last_family_discrepancy;
  memcpy(&rom_no, ow, sizeof(struct onewire_dev));

  /* set search to find the same device */
  srom->last_discrepancy = 64;
  srom->last_device_flag = false;

  if (onewire_search_rom(srom, ow)) {
    /* check if same device found */
    for (i = 0; i < 8; i++) {
      if (rom_no[i] != rom_no_in[i]) {
        
        present = false;
        break;
      }
    }
  } else {
    present = false;
  }

  /* restore state  */
  memcpy(ow, rom_no, sizeof(struct onewire_dev));
  srom->last_discrepancy = ld;
  srom->last_device_flag = ldf;
  srom->last_family_discrepancy = lfd;

  return present;
}
/*---------------------------------------------------------------------------*/
void onewire_search_rom_setup(struct search_rom *srom,
    struct onewire_dev *ow)
{
  srom->last_discrepancy = 0;
  srom->last_device_flag = false;
  srom->last_family_discrepancy = 0;
  memset(ow, 0x0, sizeof(struct onewire_dev));
  return;
}
/*---------------------------------------------------------------------------*/
void onewire_search_rom_target_setup(struct search_rom *srom,
    struct onewire_dev *ow)
{
  /* ensure only family id present */
  memset(ow + 1, 0x0, sizeof(struct onewire_dev) - 1);
  srom->last_discrepancy = 64;
  srom->last_family_discrepancy = 0;
  srom->last_device_flag = false;
  return;
}
/*---------------------------------------------------------------------------*/
int
onewire_read_rom(struct onewire_dev *ow)
{
  int i;

  PIN_INIT();

  DISABLE_INTERRUPTS();

  if (reset() == 0) {
    write_byte(READ_ROM_CMD);    /* Read ROM command. */
    ow->family = read_byte();
    for (i = 0; i < 6; i++) {
      ow->id[i] = read_byte();
    }
    ow->crc = read_byte();

    ENABLE_INTERRUPTS();

    return 1;	/* Success! */

  } else {
    ENABLE_INTERRUPTS();
  }

  memset(ow, 0x0, sizeof(struct onewire_dev));
  return 0;  /* Fail! */
}
/*---------------------------------------------------------------------------*/
int
onewire_match_rom(struct onewire_dev *ow)
{
  PIN_INIT();

  DISABLE_INTERRUPTS();

  if (reset() == 0) {
    /* Match ROM command. */
    write_byte(MATCH_ROM_CMD);

    /* Write 64-bit device ID */
    write_bytes((uint8_t *)ow, 8);

    ENABLE_INTERRUPTS();

    return 1;	/* Success! */

  } else {
    ENABLE_INTERRUPTS();
  }

  return 0;  /* Fail! */
}
/*---------------------------------------------------------------------------*/
int
onewire_skip_rom()
{
  PIN_INIT();

  DISABLE_INTERRUPTS();

  if (reset() == 0) {
    /* Skype ROM command. */
    write_byte(SKIP_ROM_CMD);

    ENABLE_INTERRUPTS();

    return 1;	/* Success! */

  } else {
    ENABLE_INTERRUPTS();
  }

  return 0;  /* Fail! */
}
/*---------------------------------------------------------------------------*/
int
onewire_read_fn_cmd_bytes(uint8_t cmd, uint8_t *buf, unsigned count)
{
  int i;

  DISABLE_INTERRUPTS();

  /* Write command */
  write_byte(cmd);

  count--;

  for (i = count; i >= 0; i--) {
    buf[count - i] = read_byte();
  }

  ENABLE_INTERRUPTS();

  return 1;	/* Success! */
}
/*---------------------------------------------------------------------------*/
int
onewire_write_bytes(uint8_t *buf, unsigned count)
{
  PIN_INIT();

  DISABLE_INTERRUPTS();

  write_bytes(buf, count);

  ENABLE_INTERRUPTS();

  return 1;	/* Success! */
}
/*---------------------------------------------------------------------------*/
int
onewire_write_cmd(uint8_t cmd)
{
  /* command. */
  write_byte((uint8_t)cmd);
  return;
}
