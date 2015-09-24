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
#include "print.h"
#include "nuts_bolts.h"
#include "planner.h"
#include "gcode.h"
#include "report.h"


// WII Extension Controller ID
// see http://wiibrew.org/wiki/Wiimote/Extension_Controllers

#define ID_NUN_CHUK_CONTROLLER 0xA4200000
#define ID_CLASSIC_CONTROLLER 0xA4200101

#define CONTROLLER_NONE 0
#define CONTROLLER_NUN_CHUK 1
#define CONTROLLER_CLASSIC 2


#define PLANNER_BLOCK_COUNT_TRESHOLD 3
#define MOTION_PLUS_ADR_ENABLE 0x53 // 0x53 << 1 = 0xA6
#define MOTION_PLUS_ADR 0x52        // 0x52 << 1 = 0xA4
#define GBUFFER_SIZE 32
#define DEFAULT_WAIT_TICKS 10
//#define DEFAULT_WAIT_TICKS 100
#define DEBOUNCE_WAIT_TICKS 30




// index 4
#define DATA_D_RIGHT 7
#define DATA_D_DOWN 6
#define DATA_LT 5
#define DATA_MINUS 4
#define DATA_HOME 3
#define DATA_PLUS 2
#define DATA_RT 1

// index 5
#define DATA_ZL 7
#define DATA_B 6
#define DATA_Y 5
#define DATA_A 4
#define DATA_X 3
#define DATA_ZR 2
#define DATA_D_LEFT 1
#define DATA_D_UP 0

// user friendly defines for buttons
#define BTN_D_UP 0
#define BTN_D_DOWN 1
#define BTN_D_LEFT 2
#define BTN_D_RIGHT 3
#define BTN_A 4
#define BTN_B 5
#define BTN_X 6
#define BTN_Y 7
#define BTN_LT 8
#define BTN_RT 9
#define BTN_MINUS 10
#define BTN_PLUS 11
#define BTN_HOME 12
#define BTN_ZL 13
#define BTN_ZR 14



volatile uint32_t ticks;
volatile uint32_t wait_ticks;
volatile uint32_t idle_ticks;

uint8_t twiBuffer[8];
char gbuffer[32];
uint8_t gbuffer_index = 0;
uint8_t min_wait_ticks = DEFAULT_WAIT_TICKS;
uint8_t controllerEnabled = 0; // jogging disabled if no controller is found
uint8_t home_button_counter = 0;

uint8_t calib_x1,calib_y1,calib_x2,calib_y2 = 0; // calibration values for joysticks

// called every 10ms
ISR(TIMER5_COMPA_vect, ISR_BLOCK) {
	ticks++;
	wait_ticks++;
	idle_ticks++;
}


void gbuffer_reset() {
	gbuffer_index = 0;
}

void gbuffer_push(char* data) {
	while (*data && gbuffer_index < GBUFFER_SIZE) {
		gbuffer[gbuffer_index++] = *data++;
	}
	gbuffer[gbuffer_index] = 0; // 0 terminated string
}


void init_systicks() {
	TCCR5B = _BV(CS52) | _BV(CS50) | _BV(WGM52); // prescale 1024 and MODE CTC
	TCCR5A = 0; // MODE CTC (see datasheet)
	TIMSK5 = (1 << OCIE5A); // enable compare match interrupt

	// 16000000/1024/156 == 100HZ -> 10 ms
	OCR5A = 155; // !!! must me set last or it will not work!
}


void print_buffer() {
	uint8_t i=0;
	for(; i < 6; i++) {
		print_uint8_base10(twiBuffer[i]);
		printString(" ");
	}
	printString("\r\n");
}


void jog_init() {

	// Initialize jog switch port bits and DDR
	JOG_DDR &= ~(_BV(JOG_SDA) | _BV(JOG_SCL)); // TWI pins as input
	JOG_PORT |= _BV(JOG_SDA) | _BV(JOG_SCL);   // Enable internal pull-up resistors. Active low operation.

	twi_init();
	init_systicks();


	// OLD WAY INIT (see http://wiibrew.org/wiki/Wiimote/Extension_Controllers#Wii_Motion_Plus)

	/*
	twiBuffer[0] = 0x40; twiBuffer[1] = 0x00;   // prepare fake encryption keyy
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 2, 1);
	_delay_us(500); // the nunchuk needs some time to process


	// read ID of extension
	twiBuffer[0] = 0xFA;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);
	print_buffer();

	// END OF OLD WAY
*/

	/*
	 * NEW WAY INIT (see http://wiibrew.org/wiki/Wiimote/Extension_Controllers#Wii_Motion_Plus)
	 */

	twiBuffer[0] = 0xF0; twiBuffer[1] = 0x55;    // active extension and disable encryption
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 2, 1);
	_delay_us(500); // the nunchuk needs some time to process

	twiBuffer[0] = 0xFB; twiBuffer[1] = 0x00;   // disable encryption
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 2, 1);
	_delay_us(500); // the nunchuk needs some time to process

	// read ID of extension
	twiBuffer[0] = 0xFA;	// register adress of identification bytes
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	_delay_us(200);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);

	// test id value of controller
	uint32_t id_value = ((uint32_t) twiBuffer[2] << 24) + ((uint32_t) twiBuffer[3] << 16) + ((uint32_t)twiBuffer[4] << 8) + ((uint32_t)twiBuffer[5]);

	//print_uint32_base10(id_value);

	controllerEnabled = CONTROLLER_NONE;
	if (id_value == ID_NUN_CHUK_CONTROLLER) {
		controllerEnabled = CONTROLLER_NUN_CHUK;
	}
	else if (id_value == ID_CLASSIC_CONTROLLER) {
		controllerEnabled = CONTROLLER_CLASSIC;
	}

	// the following is not clean but useful to see:
	// set status for report (via $$)
	report_set_controller_available(controllerEnabled);

	//print_buffer();

	// initial read out
	_delay_us(500);
	twiBuffer[0] = 0x0;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	_delay_ms(1); // the nunchuk needs some time to process
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);


	switch(controllerEnabled) {
	case CONTROLLER_NUN_CHUK : calib_x1 = twiBuffer[0]; calib_y1 = twiBuffer[1];
	}


	//print_buffer();



	/*
	 * UNKONW WAY OF INIT
	 *
	 *
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




	// retrieve identification bytes of extension

	twiBuffer[0] = 0xFA;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);


	// retrieve center value of sticks

	twiBuffer[0] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);
*/


	/*
	int centerLeftX = (int)((twiBuffer[0] & 0x3F) << 2);
	int centerLeftY = (int)((twiBuffer[1] & 0x3F) << 2);
	int centerRightX = (int)((twiBuffer[0] & 0xC0) | ((twiBuffer[1] & 0xC0) >> 2) | ((twiBuffer[2] & 0x80) >> 4));
	int centerRightY = (int)((twiBuffer[2] & 0x1F) << 3);
	*/

}

uint8_t is_button_down(uint8_t button) {
	switch(button) {
	case BTN_D_UP : return !(twiBuffer[5] & (1 << DATA_D_UP)); break;
	case BTN_D_DOWN : return !(twiBuffer[4] & (1 << DATA_D_DOWN)); break;
	case BTN_D_LEFT : return !(twiBuffer[5] & (1 << DATA_D_LEFT)); break;
	case BTN_D_RIGHT : return !(twiBuffer[4] & (1 << DATA_D_RIGHT)); break;
	case BTN_A : return !(twiBuffer[5] & (1 << DATA_A)); break;
	case BTN_B : return !(twiBuffer[5] & (1 << DATA_B)); break;
	case BTN_X : return !(twiBuffer[5] & (1 << DATA_X)); break;
	case BTN_Y : return !(twiBuffer[5] & (1 << DATA_Y)); break;
	case BTN_LT : return !(twiBuffer[4] & (1 << DATA_LT)); break;
	case BTN_RT : return !(twiBuffer[4] & (1 << DATA_RT)); break;
	case BTN_MINUS : return !(twiBuffer[4] & (1 << DATA_MINUS)); break;
	case BTN_PLUS : return !(twiBuffer[4] & (1 << DATA_PLUS)); break;
	case BTN_HOME : return !(twiBuffer[4] & (1 << DATA_HOME)); break;
	case BTN_ZL : return !(twiBuffer[5] & (1 << DATA_ZL)); break;
	case BTN_ZR : return !(twiBuffer[5] & (1 << DATA_ZR)); break;
	}
	return 0;
}

void debug_buttons() {
	if (is_button_down(BTN_D_RIGHT)) {
		printString("DR\r\n");
	}
	else if (is_button_down(BTN_D_DOWN)) {
		printString("DD\r\n");
	}
	else if (is_button_down(BTN_D_LEFT)) {
		printString("DL\r\n");
	}
	else if (is_button_down(BTN_D_UP)) {
		printString("DU\r\n");
	}
	else if (is_button_down(BTN_A)) {
		printString("A\r\n");
	}
	else if (is_button_down(BTN_B)) {
		printString("B\r\n");
	}
	else if (is_button_down(BTN_X)) {
		printString("X\r\n");
	}
	else if (is_button_down(BTN_Y)) {
		printString("Y\r\n");
	}
	else if (is_button_down(BTN_LT)) {
		printString("LT\r\n");
	}
	else if (is_button_down(BTN_RT)) {
		printString("RT\r\n");
	}
	else if (is_button_down(BTN_MINUS)) {
		printString("-\r\n");
	}
	else if (is_button_down(BTN_PLUS)) {
		printString("+\r\n");
	}
	else if (is_button_down(BTN_HOME)) {
		printString("H\r\n");
	}
	else if (is_button_down(BTN_ZL)) {
		printString("ZL\r\n");
	}
	else if (is_button_down(BTN_ZR)) {
		printString("ZR\r\n");
	}
}

uint8_t is_moved(int8_t diff) {
	if (diff > 20 || diff < -20) {
		return 1;
	}
	return 0;
}

char* calc_step_width(int8_t diff) {
	if (diff > 80) {
		return "1.0";
	}
	if (diff > 40) {
		 return "0.5";
	}
	if (diff > 20) {
		return "0.1";
	}
	if (diff < -80) {
		return "-1.0";
	}
	else if (diff < -40) {
		 return "-0.5";
	}
	else if (diff < -20) {
		 return "-0.1";
	}
	return "0.0";
}


void jogging_nun_chuk() {
	// read raw values
	twiBuffer[0] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	// this delay is very important here (should be between 200us and 1ms)
	// apparently the extension need some time to process the data
	_delay_us(500);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);

	//print_buffer();

	// debug button output
	//debug_buttons();

	uint8_t x_value = twiBuffer[0];
	uint8_t y_value = twiBuffer[1];
	int8_t x_diff = x_value - calib_x1; // can also be negative
	int8_t y_diff = y_value - calib_y1; // can also be negative


	gbuffer_reset();
	gbuffer_push("G91G0X");
	gbuffer_push(calc_step_width(x_diff));
	gbuffer_push("Y");
	gbuffer_push(calc_step_width(y_diff));
	if (is_moved(x_diff) || is_moved(y_diff)) {
		//printString(gbuffer);
		//printString("\r\n");

		gc_execute_line(gbuffer);
	}

	/*
	print_uint8_base10(twiBuffer[0]);
	printString(" - ");
	print_uint8_base10(twiBuffer[1]);
	printString("\r\n");
	*/
}


void jogging_classic() {
	// read raw values
	twiBuffer[0] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	// this delay is very important here (should be between 200us and 1ms)
	// apparently the extension need some time to process the data
	_delay_us(500);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);

	//print_buffer();

	// debug button output
	//debug_buttons();


	// handle home button separately
	// if home button is pressed don't allow jogging
	if (is_button_down(BTN_HOME)) {
		home_button_counter++;
		// if home button is pressed longer than (2 seconds)
		// set zero for xyz
		if (home_button_counter > 20) {
			gc_execute_line("G92X0Y0Z0");
			home_button_counter = 0;
		}

		// also allow quick reset for all axis
		if (is_button_down(BTN_A)) {
			gc_execute_line("G92X0Y0Z0");
			home_button_counter = 0;
		}


		if (is_button_down(BTN_D_LEFT) || is_button_down(BTN_D_RIGHT)) {
			// reset counter to prevent reset of all axis
			home_button_counter = 0;
			// reset x-axis
			gc_execute_line("G10P0L20X0");
		}

		if (is_button_down(BTN_D_UP) || is_button_down(BTN_D_DOWN)) {
			// reset counter to prevent reset of all axis
			home_button_counter = 0;
			// reset y-axis
			gc_execute_line("G10P0L20Y0");
		}

		if (is_button_down(BTN_ZL) || is_button_down(BTN_ZR)) {
			// reset counter to prevent reset of all axis
			home_button_counter = 0;
			// reset z-axis
			gc_execute_line("G10P0L20Z0");
		}

	}
	else {
		home_button_counter = 0;

		// default step width is 0.2
		char* step_width = "0.1";
		gbuffer_reset();

		// check whether accelerated step width is selected
		if (is_button_down(BTN_A)) {
			step_width = "1.0";
		}
		else if (is_button_down(BTN_B)) {
			step_width = "0.5";
		}

		if (is_button_down(BTN_D_RIGHT)) {
			gbuffer_push("G91G0X");
			gbuffer_push(step_width);
			gc_execute_line(gbuffer);
		}
		else if (is_button_down(BTN_D_DOWN)) {
			gbuffer_push("G91G0Y-");
			gbuffer_push(step_width);
			gc_execute_line(gbuffer);
		}
		else if (is_button_down(BTN_D_LEFT)) {
			gbuffer_push("G91G0X-");
			gbuffer_push(step_width);
			gc_execute_line(gbuffer);
		}
		else if (is_button_down(BTN_D_UP)) {
			gbuffer_push("G91G0Y");
			gbuffer_push(step_width);
			gc_execute_line(gbuffer);
		}
		else if (is_button_down(BTN_ZL)) {
			gc_execute_line("G91G0Z-0.2");
		}
		else if (is_button_down(BTN_ZR)) {
			gc_execute_line("G91G0Z0.2");
		}
	}
}


void jogging()  {
	if (!controllerEnabled) {
		return;
	}

	/*
	if (idle_ticks > 500) {
		min_wait_ticks = DEFAULT_WAIT_TICKS;
	}
	*/

	// only process jogging each x ticks
	if (wait_ticks > min_wait_ticks) {
		wait_ticks = 0;

		//printString("J\r\n");

		// Only process jogging when buffer es empty or partially filled
		if (plan_get_block_buffer_count() > PLANNER_BLOCK_COUNT_TRESHOLD) {
			return;
		}

		switch(controllerEnabled) {
		case CONTROLLER_NUN_CHUK : jogging_nun_chuk(); break;
		case CONTROLLER_CLASSIC : jogging_classic(); break;
		}

	}
}
