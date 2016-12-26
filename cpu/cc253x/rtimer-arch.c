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
 *
 */

/**
 * \file
 *         Hardware-dependent functions used to support the
 *         contiki rtimer module.
 *
 *         clock_init() has set our tick speed prescaler already, so we
 *         are ticking with 500 kHz freq.
 *
 *         Contiki typedefs rtimer_clock_t as unsigned short (16bit)
 *         It thus makes sense to use the 16bit timer (Timer 1)
 *
 *         This file contains an ISR and must reside in the HOME bank
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "sys/rtimer.h"
#include "sfr-bits.h"
#include "cc253x.h"
#include "sys/energest.h"

#include "dev/rtimer_clock.h"

#include "debug.h"
#include <stdio.h>

/* Sleep timer runs on the 32k RC osc. */
/* One clock tick is 7.8 ms */
#define CLOCK_TICK_VAL (32768/128)  /* 256 */
#define RTIMER_TICK_VAL 64  /* Freq ~16384Hz */
/*---------------------------------------------------------------------------*/
/* Do NOT remove the absolute address and do NOT remove the initialiser here */
__xdata __at(0x0000) static unsigned long timer_value = 0;
/*---------------------------------------------------------------------------*/
static volatile CC_AT_DATA clock_time_t rtimer_ticks = 0; /* Uptime in ticks */
static volatile CC_AT_DATA uint8_t clock_sub_ticks = 0;   /* clock sub-ticks */
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  /*
   * - Free running mode
   * - Prescale by 32:
   *   Tick Speed has been prescaled to 500 kHz already in clock_init()
   *   We further prescale by 32 resulting in 15625 Hz for this timer.
   */
  /* Initialize tick value */
  /* Initial rtimer delay = 15625. Therefore, ~32768/2 */
  timer_value = ST0;
  timer_value += ((unsigned long int)ST1) << 8;
  timer_value += ((unsigned long int)ST2) << 16;
  timer_value += RTIMER_TICK_VAL;
  ST2 = (unsigned char)(timer_value >> 16);
  ST1 = (unsigned char)(timer_value >> 8);
  ST0 = (unsigned char)timer_value;

  STIE = 1; /* IEN0.STIE interrupt enable */
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  timer_value = ST0;
  timer_value += ((unsigned long int)ST1) << 8;
  timer_value += ((unsigned long int)ST2) << 16;
  timer_value += RTIMER_TICK_VAL;
  ST2 = (unsigned char)(timer_value >> 16);
  ST1 = (unsigned char)(timer_value >> 8);
  ST0 = (unsigned char)timer_value;

  STIE = 1; /* IEN0.STIE interrupt enable */
}
/*---------------------------------------------------------------------------*/
/* avoid referencing bits, we don't call code which use them */
#pragma save
#if CC_CONF_OPTIMIZE_STACK_SIZE
#pragma exclude bits
#endif
void
rtimer_isr(void) __interrupt(ST_VECTOR)
{
  DISABLE_INTERRUPTS();
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  /*
   * Read value of the ST0:ST1:ST2, add TICK_VAL and write it back.
   * Next interrupt occurs after the current time + TICK_VAL
   */
  timer_value = ST0;
  timer_value += ((unsigned long int)ST1) << 8;
  timer_value += ((unsigned long int)ST2) << 16;
  timer_value += RTIMER_TICK_VAL;
  ST2 = (unsigned char)(timer_value >> 16);
  ST1 = (unsigned char)(timer_value >> 8);
  ST0 = (unsigned char)timer_value;

  ++rtimer_ticks;

  /* two cycles every interrupt */
  if (++clock_sub_ticks <= 128) {
    clock_sub_ticks = 0;
    /* trigger clock */
    clock_isr();
  }

  /*
#if CLOCK_CONF_STACK_FRIENDLY
  sleep_flag = 1;
#else
  if(etimer_pending()
      && (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
    etimer_request_poll();
  }
#endif
*/
  STIF = 1; /* IRCON.STIF */
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
  ENABLE_INTERRUPTS();
}
#pragma restore
unsigned long get_rtimer_val() {
  return rtimer_ticks;
}
