#include "util/constants.h"

#ifndef DEVICE_CONFIGURATION
#define DEVICE_CONFIGURATION

// CPU settings
#define TARGET_CPU m328p
#define F_CPU 16000000
#define FREQUENCY_CORRECTION 0

// ADC settings
#define OPEN_SQUELCH true
#define ADC_REFERENCE REF_3V3
// OR
//#define ADC_REFERENCE REF_5V

// Sampling & timer setup
#define CONFIG_AFSK_DAC_SAMPLERATE 9600

// AX25 settings
#define CUSTOM_FRAME_SIZE 330

// Serial settings
#define BAUD 9600
#define SERIAL_DEBUG true
#define TX_MAXWAIT 2UL

// Port settings
#if TARGET_CPU == m328p
    #define DAC_PORT PORTD
    #define DAC_DDR  DDRD
    #define LED_PORT PORTB
    #define LED_DDR  DDRB
    #define ADC_PORT PORTC
    #define ADC_DDR  DDRC
#endif

#endif
