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
#include "door.h"
#include "cc253x.h"

/*---------------------------------------------------------------------------*/

void door_init()
{
  /* Init motor pins */
  P1SEL &= ~MOTOR_A_MASK;
  P1DIR |= MOTOR_A_MASK;
  P1SEL &= ~MOTOR_B_MASK;
  P1DIR |= MOTOR_B_MASK;

  /* Init door sensors */
  P0SEL &= ~DOOR_SENSOR_TOP_MASK;
  P0DIR |= DOOR_SENSOR_TOP_MASK;
  P0SEL &= ~DOOR_SENSOR_BOTTOM_MASK;
  P0DIR |= DOOR_SENSOR_BOTTOM_MASK;

  return;
}

void door_open()
{
  MOTOR_A_PIN = 0x01;
  MOTOR_B_PIN = 0x00;
}

void door_close()
{
  MOTOR_A_PIN = 0x00;
  MOTOR_B_PIN = 0x01;
}

void door_stop()
{
  MOTOR_A_PIN = 0x00;
  MOTOR_B_PIN = 0x00;
}

door_state get_door_state()
{
  return !DOOR_SENSOR_BOTTOM_PIN | (!DOOR_SENSOR_TOP_PIN << 1);
}

door_state set_door_state(door_state state)
{
  
  door_state current_state = get_door_state();
  if (state != current_state && state != DOOR_UNKNOWN && state != DOOR_ERROR)
  {
    while (current_state != state)
    {
      switch (state)
      {
        case DOOR_OPEN:
          door_open();
          break;

        case DOOR_CLOSED:
          door_close();
          break;
      }
    
      current_state = get_door_state();
    }
  }

  return current_state;
}
