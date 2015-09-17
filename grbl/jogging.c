/*
  Copyright (c) 2015  Huy Do, huydo1@gmail.com

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
  
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "jogging.h"

void jog_init() {
// Initialize jog switch port bits and DDR
  JOG_DDR &= ~(_BV(JOG_SDA) | _BV(JOG_SCL)); // TWI pins as input
  JOG_PORT |= _BV(JOG_SDA) | _BV(JOG_SCL);   // Enable internal pull-up resistors. Active low operation.
}

void jogging()  {
}
