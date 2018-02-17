/*
 * File:   main.c
 * Author: vasuul
 *
 * Created on January 20, 2018, 8:59 PM
 */


// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSCIO_EC// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode enabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "RGBRotencInterface.h"

// Colors is what we send to the host (0 - 255)
unsigned char colors[NUM_COLORS] = {0, 0, 0, 0};

// leds are what we display on the encoder (0 - 25)
unsigned char leds[NUM_COLORS] = {DEFAULT_LED, DEFAULT_LED, DEFAULT_LED, DEFAULT_LED};

volatile unsigned char  curr_led     = 0; // Used to determine if led is on or off for PWM
volatile unsigned char  active_color = 0; // The color currently displayed
volatile unsigned short enc_counter  = 0; // Used to determine how long has passed for encoder
volatile unsigned char  fb_pressed   = 0; // Count of front button presses
volatile unsigned short fb_counter   = 0; // Time how long the front button is held down

// Uncomment this to get output on the serial port.  Note that if the serial
//  port is enabled the serial transmit of SPI will be disabled (same pin)
//#define DEBUG

#ifdef DEBUG
#define UART_SEND(x) TXREG = (x)
#else
#define UART_SEND(x)
#endif

// Global values for Heartbeat ISR routine
unsigned short hb_counter = 0;
unsigned char hb_value = 0;

// Global values for SPI ISR routine
volatile unsigned char SPI_buf[8]   = {0};
volatile unsigned char SPI_buf_indx = 0;
unsigned char SPI_next     = 0;
volatile unsigned char SPI_msg_rcvd = 0;
unsigned char check_spi_msg = 0;

void interrupt isr(void) {
    if (INTCONbits.T0IF) {
        // The timer interrupt ticks at 5KHz
        
        if(fb_counter > 0 && fb_counter < 10000) fb_counter++;

        // Saturate the encoder timer at 1s
        if (enc_counter < 5000) enc_counter++;

        // Check the heartbeat
        //if (hb_counter++ > 5000) {
        //    HB_LED_PIN = hb_value;
        //    hb_value = (unsigned) (1 - hb_value);
        //    hb_counter = 0;
        //}
        
        // Update our PWM counter
        curr_led--;

        // Check the pwm rollover
        if (curr_led == 0) {
            // We only need 25 levels of brightness
            RED_PIN = 1;
            GREEN_PIN = 1;
            BLUE_PIN = 1;
            curr_led = 25; // TODO: Make this a define
        }
        // TODO: Could put this into main to make ISR fater
        switch (active_color) {
            case RED_LED:
                if (leds[RED_LED] == curr_led) RED_PIN = 0;
                break;
            case GREEN_LED:
                if (leds[GREEN_LED] == curr_led) GREEN_PIN = 0;
                break;
            case BLUE_LED:
                if (leds[BLUE_LED] == curr_led) BLUE_PIN = 0;
                break;
            case WHITE_LED:
                if (leds[WHITE_LED] == curr_led) {
                    RED_PIN = 0;
                    BLUE_PIN = 0;
                    GREEN_PIN = 0;
                }
                break;
            case NUM_COLORS:
                // This is just off   
            default:
                // Default is to just leave them off
                break;
        }

        // Reset the interrupt flags and timer value
        TMR0L = TMR0L_init;
        INTCONbits.T0IF = 0;
    }

    // Check the MSSP (SPI) peripheral
    if (PIR1bits.SSPIF) {
        // Just get the byte we just read and load up the next one
        SPI_buf[SPI_buf_indx] = SSPBUF;
        SSPBUF = SPI_next;
        switch(SPI_buf_indx++) {
            case 0: SPI_next = fb_pressed; fb_pressed = 0; break;
            case 1: SPI_next = active_color; break;
            case 2: SPI_next = colors[0]; break;
            case 3: SPI_next = colors[1]; break;
            case 4: SPI_next = colors[2]; break;
            case 5: SPI_next = colors[3]; break;
            case 6: SPI_msg_rcvd = 1; SPI_next = 0x0; break;
            default: check_spi_msg = 1; SPI_next = SPI_SYNC; SPI_buf_indx = 0; break;
        }
        PIR1bits.SSPIF = 0;
    }
}

void main(void) {
    // Set the instruction clock to 8MHz
    OSCCON = 0x73;

    // Initialize our I/O pins    
    CMCON = CMCON_init;
    ADCON1 = ADCON1_init;
    TRISA = TRISA_init;
    TRISB = TRISB_init;
    TRISC = TRISC_init;

    // And the timer0 register values
    T0CON = T0CON_init;
    TMR0L = TMR0L_init;
    
    // Our SPI registers
    SSPSTAT = SSPSTAT_init;
    SSPCON1 = SSPCON1_init;

    // Interrupts
    INTCON = INTCON_init;
    INTCON2 = INTCON2_init;

    // Turn on interrupts (global and peripheral)
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;

    // Set our default pin values
    GREEN_PIN = 1; // 1 is off for LEDs (common anode))
    RED_PIN = 1;
    BLUE_PIN = 1;
    FBUTTON_LED_PIN = 0;

#ifdef DEBUG
    // setup the UART
    TRISC = 0x80; // outputs
    SPBRG = 25; // BAUD rate of 19200
    SPBRGH = 0;
    BAUDCON = 0x00;
    TXSTA = _TXSTA_BRGH_MASK | _TXSTA_TXEN_MASK;
    RCSTA = _RCSTA_SPEN_MASK | _RCSTA_CREN_MASK;
#endif

    // Signal and latch values to determine when a signal has changed
    //  to watch for rising/falling edges
    unsigned char bt = 0, fbt = 0, ea = 1, eb = 0;
    unsigned char lbt = 0, lfbt = 1, eat = 1;
    
    // Values used for button and encoder debouncing
    unsigned char bt_debounce = 0; // button debounce values
    unsigned char ea_debounce = 0; // Encoder debounce value
    unsigned char fbt_debounce = 0; // Front button debounce value

    active_color = NUM_COLORS; // Start off with no color    
    SSPBUF = 0x0;
    SPI_next = SPI_SYNC; // Start off with
    
    while (1) {
        // Get the status of all of our inputs
        bt = BUTTON_PIN;
        fbt = FBUTTON_PIN;
        ea = ENCA_PIN;
        eb = ENCB_PIN;

        if(check_spi_msg) {
            switch(SPI_buf[0]) {
                case 0: break; // Do nothing
                case 1:
                    active_color = SPI_buf[1] % (NUM_COLORS + 1);
                    colors[0] = SPI_buf[2];
                    colors[1] = SPI_buf[3];
                    colors[2] = SPI_buf[4];
                    colors[3] = SPI_buf[5];
                    break;
                case 2: FBUTTON_LED_PIN = (SPI_buf[1] == 0) ? 0 : 1; break;
                case 3: FAN_PIN = (SPI_buf[1] == 0) ? 0 : 1; break;
                default: break;
            }
            check_spi_msg = 0;
        }

        // Check if we have a message to process
        if (SPI_msg_rcvd != 0) {
            SPI_msg_rcvd = 0;
        }

        // Check the encoder button and debounce it
        if (bt != lbt) {
            if (bt_debounce++ > BUTTON_DEBOUNCE) {
                // Button transitioned
                bt_debounce = 0;
                lbt = bt;

                if (!bt && active_color != NUM_COLORS) {
                    // Button press finished - go to next color unless we are off
                    active_color = (unsigned char) ((active_color + 1) % NUM_COLORS);
                    UART_SEND(active_color);
                }
            } else {
                /* Just keep going */
            }
        } else bt_debounce = 0;

        // Then the front button
        if (fbt != lfbt) {
            if (fbt_debounce++ > BUTTON_DEBOUNCE) {
                lfbt = fbt;
                fbt_debounce = 0;
                
                if(fbt) {
                    // Button released
                    if(fb_counter >= 10000) fb_pressed = 255;
                    else if(fb_pressed < 250) fb_pressed++;
                    
                    fb_counter = 0;
                } else {
                    // button pressed, start counting
                    fb_counter = 1;
                }
            } else {
            }
        } else fbt_debounce = 0;

        // Then the encoder
        //  Basically we just watch for the falling edge of A
        //  Then we sample the state of B and use that to determine
        //   the direction.  We could watch for the rising/falling edge
        //   of both to get 4x ticks per revolution, but we don't really 
        //   need that.
        if (ea != eat) {
            if (ea_debounce++ > ENCODER_DEBOUNCE) {

                ea_debounce = 0;
                eat = ea;

                if (!ea && active_color < NUM_COLORS) {
                    // Falling edge - have to decode
                    unsigned char inc_val = 0;
                    unsigned short tec = enc_counter;
                    enc_counter = 0;
                    short tmp = colors[active_color];

                    // Split the timespan up into regions and base our update on that
                    if (tec > 1000) {
                        inc_val = 1;
                    } else if (tec > 50) {
                        inc_val = 3;
                    } else {
                        inc_val = 9;
                    }

                    UART_SEND(inc_val);

                    if (!eb) {
                        // Turned one way
                        tmp += inc_val;
                        if (tmp > MAX_ENCODER) tmp = MAX_ENCODER;
                    } else {
                        tmp -= inc_val;
                        if (tmp < MIN_ENCODER) tmp = MIN_ENCODER;
                    }

                    colors[active_color] = (unsigned char) tmp;

                    // Leave the active color fixed since it is now just a
                    //  dial position indicator, not a brightness indicator
                    leds[active_color] = DEFAULT_LED; //(unsigned char)(tmp / 10); // gets us from 0-255 to 0-25
                }
            } else {
            }
        } else ea_debounce = 0;
    } // end infinite while
    return;
}
