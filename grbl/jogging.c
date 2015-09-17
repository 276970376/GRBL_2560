/*
  Copyright (c) 2015  Huy Do, huydo1@gmail.com

  Wii Classic Controller part inspired by: http://www.instructables.com/id/USB-Wii-Classic-Controller/

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
#include "twi.h"


// index 4
#define BTN_D_RIGHT 7
#define BTN_D_DOWN 6
#define BTN_LT 5
#define BTN_MINUS 4
#define BTN_HOME 3
#define BTN_PLUS 2
#define BTN_RT 1

// index 5
#define BTN_ZL 7
#define BTN_B 6
#define BTN_Y 5
#define BTN_A 4
#define BTN_X 3
#define BTN_ZR 2
#define BTN_D_LEFT 1
#define BTN_D_UP 0


uint8_t twiBuffer[8];

void jog_init() {
	// Initialize jog switch port bits and DDR
	JOG_DDR &= ~(_BV(JOG_SDA) | _BV(JOG_SCL)); // TWI pins as input
	JOG_PORT |= _BV(JOG_SDA) | _BV(JOG_SCL);   // Enable internal pull-up resistors. Active low operation.


	// initialize the Wii Classic Controller
	// make decryption predictable

	twiBuffer[0] = 0x40; twiBuffer[1] = 0x00; twiBuffer[2] = 0x00; twiBuffer[3] = 0x00; twiBuffer[4] = 0x00; twiBuffer[5] = 0x00; twiBuffer[6] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 7, 1);
	_delay_us(500); // the nunchuk needs some time to process

	twiBuffer[0] = 0x46; twiBuffer[1] = 0x00; twiBuffer[2] = 0x00; twiBuffer[3] = 0x00; twiBuffer[4] = 0x00; twiBuffer[5] = 0x00; twiBuffer[6] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 7, 1);
	_delay_us(500); // the nunchuk needs some time to process

	twiBuffer[0] = 0x4C; twiBuffer[1] = 0x00; twiBuffer[2] = 0x00; twiBuffer[3] = 0x00; twiBuffer[4] = 0x00; twiBuffer[5] = 0x00; twiBuffer[6] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 5, 1);
	_delay_us(500); // the nunchuk needs some time to process

	// retrieve center value of sticks

	twiBuffer[0] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);

	/*
	int centerLeftX = (int)((twiBuffer[0] & 0x3F) << 2);
	int centerLeftY = (int)((twiBuffer[1] & 0x3F) << 2);
	int centerRightX = (int)((twiBuffer[0] & 0xC0) | ((twiBuffer[1] & 0xC0) >> 2) | ((twiBuffer[2] & 0x80) >> 4));
	int centerRightY = (int)((twiBuffer[2] & 0x1F) << 3);
	*/

}

uint8_t is_button_down(uint8_t index, uint8_t button_bit) {
	return (!(twiBuffer[index] & (1 << button_bit)));
}

void jogging()  {

	// read raw values
	twiBuffer[0] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);

	if (is_button_down(4, BTN_D_RIGHT)) {

	}
	else if (is_button_down(4, BTN_D_DOWN)) {

	}
	else if (is_button_down(5, BTN_D_LEFT)) {

	}
	else if (is_button_down(5, BTN_D_UP)) {

	}
}
