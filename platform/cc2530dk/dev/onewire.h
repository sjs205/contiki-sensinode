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

#ifndef ONEWIRE_H
#define ONEWIRE_H

#include <stdbool.h>

struct onewire_dev {
  uint8_t family;
  uint8_t id[6];
  uint8_t crc;
};

struct search_rom {
  uint8_t last_discrepancy;
  uint8_t last_family_discrepancy;
  uint8_t last_device_flag;
};

typedef enum {
  OW_MEMORY,
  OW_SWITCH,
  OW_TEMP
} ow_type;

//extern struct onewire_dev ow_dev;
extern int onewire_init();
extern int onewire_read_rom(struct onewire_dev *ow);
extern int onewire_skip_rom();
extern int onewire_match_rom(struct onewire_dev *ow);
extern int onewire_read_fn_cmd_bytes(uint8_t cmd, uint8_t *buf,
    unsigned count);
extern int onewire_write_bytes(uint8_t *buf, unsigned count);
extern int onewire_write_cmd(uint8_t cmd);

extern int onewire_search_rom(struct search_rom *srom,
    struct onewire_dev *ow);
extern void onewire_search_rom_setup(struct search_rom *srom,
    struct onewire_dev *ow);
extern int onewire_search_rom_verify(struct search_rom *srom,
    struct onewire_dev *ow);
extern void onewire_search_rom_target_setup(struct search_rom *srom,
    struct onewire_dev *ow);

#endif /* ONEWIRE_H */
