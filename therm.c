/*
 * RHT03 Sensor driver for AVR 
 * Author: Karl Matthias
 *
 * Supports the RHT03 temperature/humidity sensor
 *
 * NOTES: Doesn't use hardware interrupts or the clock: spinwaits on changes to
 * the pin. Does not protect against requesting readings too quickly from the
 * sensor. The data sheet says 1 reading every 2 seconds is what is supported. You
 * may get strange results if you query more often than that.
 * 
 * The current settings assume an 8Mhz clock. You cannot run this at 1Mhz due to
 * the sampling rate being too short. You can run it at 4Mhz with the prescaler
 * change and by setting LONG_PULSE_LENGTH = 14.
 * 
 * DATASHEET: http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Weather/RHT03.pdf
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"

#define DEBUG      1

#define LED        PB1
#define LED_PORT   PORTB
#define THERM      PB0
#define THERM_PORT PORTB

#define BIT_TRANSITIONS   40 + 2 // bits + preamble
#define LONG_PULSE_LENGTH 28     // 28 tries @ 8mhz

#if DEBUG
uint8_t tries_b[43];
uint8_t try_count = 0;
#endif

typedef struct { 
	uint16_t temperature;
	uint16_t humidity;
	uint8_t  checksum;
} RHTresult;

void blink(int times) {
    for(int i = 1; i <= times; i++) {
        LED_PORT ^= (1 << LED);
        _delay_ms(100);
        LED_PORT ^= (1 << LED);
        _delay_ms(100);
    }
}

uint8_t read_pin(void) {
    return (PINB & (1 << THERM)) ? 1 : 0;
}

void print_debug(RHTresult *result) {
#if DEBUG
	printString("Bits:\r\n");
	for(int i = 0; i < 43; i++) { 
		printByte(i);
		printString(": ");
		printByte(tries_b[i]);
		printString("\r\n"); 
	}

	printString("\r\n\r\n");
	printString("Temperature: ");
	printWord(result->temperature);
	printString("\r\n");
	printString("Humidity: ");
	printWord(result->humidity);
	printString("\r\n");
	printString("Checksum: ");
	printByte(result->checksum);
	printString("\r\n\r\n");
#endif
}

void next_bit(uint8_t *state) {
	uint8_t tries = 0;

	// Pulses are all high pulses, low means nothing
	while(read_pin() == 0) { }

	// Sample all the meaningful pulses
	while(read_pin() == 1) {
		if(tries++ == 255) {
			break; // Don't get stuck looping forever
		}
	}
	// 1 is a long duration high pulse, 0 is a short one
	*state = (tries > LONG_PULSE_LENGTH) ? 1 : 0;

#if DEBUG
	tries_b[try_count] = tries;
	try_count++;
#endif
}

void read_therm(RHTresult *result) {
	DDRB |= (1 << THERM);   // Output mode

	// Drop it low for 10ms
	THERM_PORT &= ~(1 << THERM); // Pin low
	_delay_ms(5);                // Wait for the sensor to init
	cli();
	THERM_PORT |= (1 << THERM);  // Pin high

	// We're going to read from now on
	DDRB &= ~(1 << THERM);  // Input mode

	// Keep some state
	uint8_t bits[5] = { 0, 0, 0, 0, 0 };
	uint8_t state = 0;
	uint8_t bucket;

	// Ignore 2-bit pre-amble
	next_bit(&state);
	next_bit(&state);

	// Begin scanning for transitions
	for(int counter=0; counter < BIT_TRANSITIONS; counter++) {
		next_bit(&state);
		bucket = counter >> 3; // bit-shift-divide by 8
		bits[bucket] = (bits[bucket] << 1) | state;
	}

	result->humidity    = (bits[0] << 8) | bits[1];
	result->temperature = (bits[2] << 8) | bits[3];
	result->checksum    = (bits[0] + bits[1] + bits[2] + bits[3] == bits[4]) ? 1 : 0;

	print_debug(result);
	sei();
}

void setup_avr(void) {
	// Run at 8Mhz
	CLKPR = (1 << CLKPCE); // Enable changing clock prescaler
	CLKPR = 0;             // Set to 8Mhz (no divider)

	// Set port directions to output
    DDRB |= (1 << LED);

#if DEBUG
	initUSART();
	printString("Therm running\r\n");
#endif
}

int main (void) {
	setup_avr();
	RHTresult result;
	while(1) {
		blink(3);
		_delay_ms(2000); // Sensor needs to boot
		read_therm(&result);
	}
}
