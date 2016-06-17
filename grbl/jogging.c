/*

  Copyright(c) 2016 Huy Do, huydo1@gmail.com

  Jogging part based on work from Carsten Meyer (c't Make), cm@ct.de
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
#include "stepper.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "gcode.h"
#include "config.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "planner.h"
#include "protocol.h"
#include "limits.h"
#include "jogging.h"
#include "twi.h"
#include "serial.h"
#include "report.h"
#include <avr/pgmspace.h>
#include "report.h"
#include "print.h"


#define JOG_SPEED 0x150


// WII Extension Controller ID
// see http://wiibrew.org/wiki/Wiimote/Extension_Controllers

#define ID_CLASSIC_CONTROLLER 0xA4200101

#define CONTROLLER_NONE 0
#define CONTROLLER_NUN_CHUK 1
#define CONTROLLER_CLASSIC 2


#define PLANNER_BLOCK_COUNT_TRESHOLD 3
#define MOTION_PLUS_ADR_ENABLE 0x53 // 0x53 << 1 = 0xA6
#define MOTION_PLUS_ADR 0x52        // 0x52 << 1 = 0xA4
#define GBUFFER_SIZE 32
//#define DEFAULT_WAIT_TICKS 10
#define DEFAULT_WAIT_TICKS 2


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

uint16_t led_count = 0;
uint8_t led_toggle;
uint32_t dest_step_rate, step_rate; // Step delay after pulse 
static uint8_t jog_bits, jog_bits_old, jog_exit, last_sys_state;
static uint8_t jstep_bits;
static uint8_t jdir_bits;
static uint8_t reverse_flag;
static uint8_t jog_select;
static uint16_t skip_max;
static volatile long work_position;
static volatile uint16_t skip_count;
static uint32_t step_delay;
static uint8_t limit_state;


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


void jog_init() {

	// Initialize jog switch port bits and DDR

#ifdef LED_PRESENT
	LED_DDR |= ((1<<LED_RUN_BIT) | (1<<LED_ERROR_BIT));
	LED_PORT |= ((1<<LED_RUN_BIT) | (1<<LED_ERROR_BIT)); // active low, so set high
#endif

	JOGSW_DDR &= ~(JOGSW_MASK); // Set as input pins
	JOGSW_PORT |= (JOGSW_MASK); // Enable internal pull-up resistors. Active low operation.

	dest_step_rate = JOG_SPEED;

	// Initialize jog switch port bits and DDR
	JOG_DDR &= ~(_BV(JOG_SDA) | _BV(JOG_SCL)); // TWI pins as input
	JOG_PORT |= _BV(JOG_SDA) | _BV(JOG_SCL);   // Enable internal pull-up resistors. Active low operation.

	twi_init();
	init_systicks();

	/*
	 * NEW WAY INIT (see http://wiibrew.org/wiki/Wiimote/Extension_Controllers#Wii_Motion_Plus)
	 */

	twiBuffer[0] = 0xF0; twiBuffer[1] = 0x55;    // active extension and disable encryption
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 2, 1);
	_delay_us(1000); // the nunchuk needs some time to process

	twiBuffer[0] = 0xFB; twiBuffer[1] = 0x00;   // disable encryption
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 2, 1);
	_delay_us(1000); // the nunchuk needs some time to process

	// read ID of extension
	twiBuffer[0] = 0xFA;	// register adress of identification bytes
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	_delay_us(500);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);

	// test id value of controller
	uint32_t id_value = ((uint32_t) twiBuffer[2] << 24) + ((uint32_t) twiBuffer[3] << 16) + ((uint32_t)twiBuffer[4] << 8) + ((uint32_t)twiBuffer[5]);

	// DEBUG
	//print_uint32_base10(id_value);

	controllerEnabled = CONTROLLER_NONE;
	if (id_value == ID_CLASSIC_CONTROLLER) {
		controllerEnabled = CONTROLLER_CLASSIC;
	}

	// the following is not clean but useful to see:
	// set status for report (via $$)
	// report_set_controller_available(controllerEnabled);

	//print_buffer();

	// initial read out
	_delay_us(1000);
	twiBuffer[0] = 0x0;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);
	_delay_ms(1); // the nunchuk needs some time to process
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);



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


uint8_t read_jog_bits_digital() {
	// read from digital buttons
	return 	(~JOGSW_PIN) & JOGSW_MASK; // active low
}

uint8_t read_jog_bits_wii() {
	// read from controller
	// if there is no controller return 0
	if (!controllerEnabled) {
		return 0;
	}
	uint8_t jbits = 0;

	// read raw values
	twiBuffer[0] = 0x00;
	twi_writeTo(WIIEXT_TWI_ADDR, twiBuffer, 1, 1);

	// this delay is very important here (should be between 200us and 1ms)
	// apparently the extension need some time to process the data
	// _delay_us(500);
	_delay_ms(1);
	twi_readFrom(WIIEXT_TWI_ADDR, twiBuffer, 6);


	// only jog if button A (accellerate) is pressed
	if (!is_button_down(BTN_A)) {
		return 0;
	}

	// set jbits to be compatible with original jogging routines
	if (is_button_down(BTN_D_LEFT)) {
		jbits |= 1 << JOGREV_X_BIT;
	}
	if (is_button_down(BTN_D_RIGHT)) {
		jbits |= 1 << JOGFWD_X_BIT;
	}
	if (is_button_down(BTN_D_DOWN)) {
		jbits |= 1 << JOGREV_Y_BIT;
	}
	if (is_button_down(BTN_D_UP)) {
		jbits |= 1 << JOGFWD_Y_BIT;
	}
	if (is_button_down(BTN_RT)) {
		jbits |= 1 << JOGREV_Z_BIT;
	}
	if (is_button_down(BTN_LT)) {
		jbits |= 1 << JOGFWD_Z_BIT;
	}
	// DEBUG
	/*
	if (jbits >  0) {
		print_uint32_base10(jbits);
		printString("\r\n");
	}
	*/
	return jbits;
}

void jogpad_check()
// Tests jog port pins and moves steppers
{
	jstep_bits = 0;
	jdir_bits = 0;
	reverse_flag = 0;
	jog_select = 0;
	jog_bits = 0;

#ifdef LED_PRESENT
	switch (sys.state) {
		case STATE_ALARM: case STATE_SAFETY_DOOR:
		LED_PORT &= ~(1<<LED_ERROR_BIT);
		LED_PORT &= ~(1<<LED_RUN_BIT);
		break;
		case STATE_HOLD: case STATE_MOTION_CANCEL:
		if (led_toggle) {
			LED_PORT &= ~(1<<LED_ERROR_BIT);
			LED_PORT |= (1<<LED_RUN_BIT);
		} else {
			LED_PORT &= ~(1<<LED_ERROR_BIT);
			LED_PORT &= ~(1<<LED_RUN_BIT);
		}
		break;
		case STATE_CYCLE: case STATE_HOMING:
		LED_PORT &= ~(1<<LED_RUN_BIT);
		LED_PORT |= (1<<LED_ERROR_BIT);
		return;
		case STATE_IDLE:
		LED_PORT |= ((1<<LED_RUN_BIT) | (1<<LED_ERROR_BIT));
	}

	led_count += 1;
	if (led_count > 7000) {
		led_toggle = ~led_toggle;
		led_count = 0;
	}
#endif


	last_sys_state = sys.state;

	jog_bits = read_jog_bits_wii();
	if (!jog_bits) {
		return;
	}  // nothing pressed

	sys.state = STATE_JOG;

	// check for reverse switches
	if (jog_bits & (1 << JOGREV_X_BIT)) { // X reverse switch on
		jdir_bits ^= (1 << X_DIRECTION_BIT);
		jstep_bits = jdir_bits ^ (1 << X_STEP_BIT);
		reverse_flag = 1;
	}
	if (jog_bits & (1 << JOGREV_Y_BIT)) { // Y reverse switch on
		jdir_bits ^= (1 << Y_DIRECTION_BIT);
		jstep_bits = jdir_bits ^ (1 << Y_STEP_BIT);
		reverse_flag = 1;
		jog_select = 1;
	}
	if (jog_bits & (1 << JOGREV_Z_BIT)) { // Z reverse switch on
		jdir_bits ^= (1 << Z_DIRECTION_BIT);
		jstep_bits = jdir_bits ^ (1 << Z_STEP_BIT);
		reverse_flag = 1; // positive Z dir!
		jog_select = 2;
	}

	// check for forward switches
	if (jog_bits & (1 << JOGFWD_X_BIT)) { // X forward switch on
		jstep_bits = jdir_bits ^ (1 << X_STEP_BIT);
	}
	if (jog_bits & (1 << JOGFWD_Y_BIT)) { // Y forward switch on
		jstep_bits = jdir_bits ^ (1 << Y_STEP_BIT);
		jog_select = 1;
	}
	if (jog_bits & (1 << JOGFWD_Z_BIT)) { // Z forward switch on
		jstep_bits = jdir_bits ^ (1 << Z_STEP_BIT);
		// reverse_flag = 1; // positive Z dir!
		jog_select = 2;
	}

	// (JOG_MAX_SPEED-JOG_MIN_SPEED)/256
	uint32_t max_frequ;

	uint32_t jog_speed_fac;

	max_frequ = (settings.max_rate[jog_select] * settings.steps_per_mm[jog_select]) / 60;// max_rate war in mm/min, max_freq in Hz
	jog_speed_fac = (max_frequ - JOG_MIN_SPEED)/256;

	dest_step_rate = JOG_SPEED;
	dest_step_rate = (dest_step_rate * jog_speed_fac) + JOG_MIN_SPEED;

	step_rate = JOG_MIN_SPEED;   // set initial step rate
	jog_exit = 0;


	jog_bits_old = jog_bits;
	//uint8_t i = 0;  // now index for sending position data

	float mm_per_step;

	if (bit_istrue(settings.flags, BITFLAG_REPORT_INCHES)) {
		mm_per_step = 1 / (settings.steps_per_mm[jog_select] * INCH_PER_MM);
	} else {
		mm_per_step = 1 / settings.steps_per_mm[jog_select];
	}

	work_position = sys.position[jog_select];  // Ausgangswert
	plan_reset(); // Reset planner buffer to zero planner current position and to clear previous motions.

	stepper_init();  // initialisiert Timer 1 OC
	TIMSK0 &= ~(1 << OCIE0A); // Disable Timer0 Compare Match A interrupt

	TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11))) | (1 << CS10); // Set in st_go_idle().
	TIMSK1 &= ~(1 << OCIE1A);  // Timer 1 zunaechst abgeschaltet

#ifdef LED_PRESENT
	LED_PORT &= ~(1<<LED_RUN_BIT);
#endif

	uint8_t idx;
	uint8_t step_port_invert_mask = 0;
	uint8_t dir_port_invert_mask = 0;
	for (idx = 0; idx < N_AXIS; idx++) {
		if (bit_istrue(settings.step_invert_mask, bit(idx))) {
			step_port_invert_mask |= get_step_pin_mask(idx);
		}
		if (bit_istrue(settings.dir_invert_mask, bit(idx))) {
			dir_port_invert_mask |= get_direction_pin_mask(idx);
		}
	}

	DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK)
			| ((jdir_bits ^ dir_port_invert_mask) & DIRECTION_MASK);
	STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK); // noch nicht aktiv

	delay_us(settings.pulse_microseconds);

	// Enable stepper drivers.
	if (bit_istrue(settings.flags, BITFLAG_INVERT_ST_ENABLE)) {
		STEPPERS_DISABLE_PORT |= (1 << STEPPERS_DISABLE_BIT);
	} else {
		STEPPERS_DISABLE_PORT &= ~(1 << STEPPERS_DISABLE_BIT);
	}

	jog_active = true;	// set jogging flag, -cm

	skip_count = 0;
	skip_max = 8;			// Prescaler nicht aendern, deshalb so
	limit_state = limits_get_state(); //  LIMIT_MASK & (LIMIT_PIN ^ settings.invert_mask);
	step_delay = 10000;
	OCR1A = step_delay;
	TIMSK1 |= (1 << OCIE1A);  // Timer 1 starten

	wait_ticks = 0;
	for (;;) { // repeat until button/joystick released

		// Get limit pin state
		limit_state = limits_get_state(); //  LIMIT_MASK & (LIMIT_PIN ^ settings.invert_mask);

		// only read new value after every x ticks
		if (wait_ticks > min_wait_ticks) {
			wait_ticks = 0;
			jog_bits = read_jog_bits_wii(); // neu abfragen weil in der Schleife
		}

		step_delay = (1710000 / step_rate); //

		// Maximaler Wert fuer 16-Bit-Timer darf nicht ueberschritten werden
		if (step_delay > 65535) {
			step_delay = 65535;
		}

		// jog_exit wird im ISR gesetzt sobald Stopp erreicht
		if (jog_exit || (sys_rt_exec_state & EXEC_RESET)) {
			TIMSK1 &= ~(1 << OCIE1A);	// Timer disable
			jog_active = false;	// reset jogging flag, -cm

#ifdef LED_PRESENT
			LED_PORT |= (1<<LED_RUN_BIT);
#endif

			st_reset();
			stepper_init();
			sys.state = last_sys_state;
			sys.position[jog_select] = work_position;
			plan_sync_position();  	//	sys.position --> pl.position
			gc_sync_position(); //	system_convert_array_steps_to_mpos(gc_state.position,sys.position);
			return;
		}

		// TODO update messate format (seems to be from c't)
		// currently: <JogY,168.135>
		// or: <JogX,-118.484>
		//  report_realtime_status()  benoetigt viel Zeit, deshalb Minimalmeldung
		/*
		if (sys_rt_exec_state & EXEC_STATUS_REPORT) {
			// status report requested, print short msg only
			printPgmString(PSTR("<Jog"));
			serial_write(88 + jog_select); // 88 = X + 1 = Y etc.
			serial_write(44);
			// Relativposition zum Werkstueck-Nullpunkt ermitteln aud ausgeben
			printFloat_CoordValue(
					work_position * mm_per_step
							- gc_state.coord_offset[jog_select]);
			serial_write(62); // ">" char
			serial_write(13);
			serial_write(10);
			sys_rt_exec_state = 0;
		}
		*/
	}
}

// ISR(TIMER1_COMPA_vect) wird hierhin umgeleitet wenn jog_active true ist
// Interupt setzt eigenen Counterwert neu und berechnet gewuenschte Steprate.
void jog_isr() {
	if (skip_count == 0) {

		OCR1A = step_delay; // Counter ist jetzt auf Null, darf neu gesetzt werden

		if (limit_state && reverse_flag) {
			jog_exit = 1;
		} // immediate stop on any switch

		if (jog_bits == jog_bits_old) { // nothing changed
			if (step_rate < (dest_step_rate - 5)) { // Hysterese fuer A/D-Wandlung
				step_rate += JOG_RAMP; // accellerate
			}
			if (step_rate > (dest_step_rate + 5)) { // Hysterese fuer A/D-Wandlung
				step_rate -= JOG_RAMP; // brake
			}
		} else {
			if (step_rate > (JOG_MIN_SPEED)) { // fast brake to complete stop
				step_rate = step_rate - 10;
			} else {
				jog_exit = 1;
			} // finished to stop and exit
//			if (step_rate > (JOG_MIN_SPEED * 2)) { // fast brake to complete stop
//				step_rate = ((step_rate * 99) / 100) - JOG_MIN_SPEED;
//			} else {
//				jog_exit = 1;
//			} // finished to stop and exit
		}

		// Set stepping pins, nochmal invertieren
		STEP_PORT = STEP_PORT ^ (jstep_bits & STEP_MASK);

		if (reverse_flag) {
			work_position -= 1;
		} else {
			work_position += 1; // relative print_position in mm since last report
		}
		skip_count = skip_max;

		// Delay fuer Step-Impuls im Interrupt, nicht so sehr schoen, aber einfach
		delay_us(settings.pulse_microseconds);

		// Reset stepping pins, nochmal invertieren
		STEP_PORT = STEP_PORT ^ (jstep_bits & STEP_MASK);

	} else {
		skip_count -= 1;
	}
}

