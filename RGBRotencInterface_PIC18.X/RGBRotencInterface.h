/* 
 * File:   RGBRotencInterface.h
 * Author: vasuul
 * Comments: Main header file for the project
 * Revision history: 
 */

#ifndef RGB_ROTENC_INTERFACE_H
#define	RGB_ROTENC_INTERFACE_H

#define ADCON1_init 0x0F // Set PORTA/PORTB pins to digital 

// For SPI, SCK and SDI are input (RB1/RB0)
// For SPI, SDO is output (RC7)

// Setup our pins - We need
// 4 inputs
//  Button and FrontButton
//  Encoder A,B
#define TRISB_init 0xFF // All input
#define ENCA_PIN    PORTBbits.RB6
//#define ENCB_PIN    PORTBbits.RB5
#define BUTTON_PIN  PORTBbits.RB4
#define FBUTTON_PIN PORTBbits.RB3

// 5 outputs
//  RGB LEDs
//  FrontButton LED
//  Heartbeat LED
#define TRISA_init  0x30 // All output except RA5(_SS) and RA4
#define CMCON_init  0x07 // Comparators off on port A
#define RED_PIN          PORTAbits.RA0
#define GREEN_PIN        PORTAbits.RA1
#define BLUE_PIN         PORTAbits.RA2
#define FBUTTON_LED_PIN  PORTAbits.RA3
//#define HB_LED_PIN       PORTAbits.RA4
#define ENCB_PIN         PORTAbits.RA4

// All output for now.  If we need RC4 or RC5 as digital I/O
//  we need to turn off the USB.  They can only be inputs
//  Keep that in mind
#define TRISC_init 0x00 // All output for now
#define FAN_PIN          PORTCbits.RC2

// All the defines to setup the SPI interface
//  SPI mode 0 => CKP = 0, CKE = 1
#define SSPSTAT_init 0x40; // CKE = 1
#define SSPCON1_init 0x24; // CKP = 0, enable SPI and overflow

// We want the ISR to tick at 5KHz
// 1:2 pre-scaler, 8-bit counter, enabler timer0
//  8MHz / 4 => 2MHz for instruction clock
//  2MHz / 2 => 1MHz for timer clock
//  1MHz / 5KHz => 200, so we need to tick 200 times per interrupt
//   Which means we set the timer value to 55 (255 - 200)
#define T0CON_init 0xC0
#define TMR0L_init 55

// Enable Timer0 interrupt and set it high priority
//  INTCON2 also has the diable for PORTB pull up
#define INTCON_init  0x20
#define INTCON2_init 0x84

// Enable SPI interrupts
#define PIE1_init 0x08 // Enable MSSP interrupt
#define IPR1_init 0x08 // Set MSSP interupt to high priority

#define BUTTON_DEBOUNCE 20
#define ENCODER_DEBOUNCE 20

#define RED_LED    0
#define GREEN_LED  1
#define BLUE_LED   2
#define WHITE_LED  3
#define NUM_COLORS 4

#define DEFAULT_LED 15

#define MAX_ENCODER 255
#define MIN_ENCODER 0

#define SPI_SYNC 0x5A

#endif	/* RGB_ROTENC_INTERFACE_H */

