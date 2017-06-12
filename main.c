// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = NSLEEP    // Watchdog Timer Enable (WDT enabled while running and disabled in Sleep)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = ON       // Low-Power Brown Out Reset (Low-Power BOR is enabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

#define _XTAL_FREQ 1000000L
#define TIMER2_FREQ _XTAL_FREQ/64
#define PWM_PERIOD TIMER2_FREQ/(1000/5)/4

#include <xc.h>
#include <stdint.h>

// TODO:
// Battery sense.
// Sleep mode.
// Better front light flash pattern.
// Detect full battery (RA5 goes high again?).

// RA0 -- ICSPDAT
// RA1 -- ICSPCLK
// RA2 -- Button (LOW=pressed)
// RA3 -- ~MCLR
// RA4 -- Battery sense (Vbatt/2)  (AN3)
// RA5 -- Low when charging? High otherwise.
// RC0 -- Down LEDs
// RC1 -- Green LED
// RC2 -- Red LED
// RC3 -- Orange LED
// RC4 -- Blue LED
// RC5 -- Front LEDs

// Tristate -- HIGH = input, LOW = output

void main(void) {
    // 1 MHz clock.
    OSCCONbits.SCS = 0;
    OSCCONbits.IRCF = 0b1011;
    
    // high-z the programming pins.
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA3 = 1;
    
    // high-z button
    TRISAbits.TRISA2 = 1;
    ANSELAbits.ANSA2 = 0;
    // high-z, analog, no-pull-up batt sense
    TRISAbits.TRISA4 = 1;
    ANSELAbits.ANSA4 = 1;
    WPUAbits.WPUA4 = 0;
    // high-z charge sense
    TRISAbits.TRISA5 = 1;
    
    // Output LEDs
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;
    
    // All LEDs off
    LATCbits.LATC0 = 0;
    LATCbits.LATC1 = 0;
    LATCbits.LATC2 = 0;
    LATCbits.LATC3 = 0;
    LATCbits.LATC4 = 0;
    LATCbits.LATC5 = 0;
    
    // Front LED PWM
    PWM1CONbits.PWM1EN = 0;
    PWM1CONbits.PWM1OE = 0;
    PWM1DCLbits.PWM1DCL = 0;
    PWM1DCHbits.PWM1DCH = 0;
    // Reset Timer2
    T2CON = 0;
    PR2bits.PR2 = PWM_PERIOD - 1;
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11; // 64:1
    T2CONbits.TMR2ON = 1;
    while (PIR1bits.TMR2IF == 0);
    
    // Disable ADC interrupt.
    PIE1bits.ADIE = 0;
    // Use fRC for ADC clock.
    ADCON1bits.ADCS = 0b111;
    // Configure voltage reference using VDD
    ADCON1bits.ADPREF = 0;
    // Select ADC input channel (RA4/AN3)
    ADCON0bits.CHS = 0b00011;
    // Select result format right justified
    ADCON1bits.ADFM = 1;
    // Turn on ADC module
    ADCON0bits.ADON = 1;
    
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE == 1);
    PIR1bits.ADIF = 0;
    
    // Flash all the LEDs for a power on / reset indication.
    LATCbits.LATC1 = 1;
    LATCbits.LATC2 = 1;
    LATCbits.LATC3 = 1;
    LATCbits.LATC4 = 1;
    __delay_ms(200);
    LATCbits.LATC1 = 0;
    LATCbits.LATC2 = 0;
    LATCbits.LATC3 = 0;
    LATCbits.LATC4 = 0;
    
    uint16_t counter = 0;
    uint8_t button_down = 0;
    uint8_t power_mode = 0;
    uint8_t bat_counter = 0;
    uint8_t dim_mode = 0;
    uint8_t night_mode = 1;
    
    while (1) {
        __delay_ms(10);
        CLRWDT();
        
        if (PORTAbits.RA5 == 0) {
            // Charging - turn off lights and flash Green LED at 0.5 Hz.
            power_mode = 0;
            
            // Turn off everything.
            PWM1CONbits.PWM1EN = 0;
            PWM1CONbits.PWM1OE = 0;
            LATCbits.LATC0 = 0;  // Down
            LATCbits.LATC2 = 0;  // Red
            LATCbits.LATC3 = 0;  // Orange
            LATCbits.LATC4 = 0;  // Blue
            LATCbits.LATC5 = 0;  // Front
            
            // Flash green.
            if (counter > 200) {
                LATCbits.LATC1 = 0;  // Green
                counter = 0;
            } else if (counter > 100) {
                LATCbits.LATC1 = 1;  // Green
            }
        } else {
            // Not charging - turn off green LED.
            LATCbits.LATC1 = 0;  // Green
            
            // Detect off->on button sequences.
            if (PORTAbits.RA2 == 0) {
                if (!button_down) {
                    // Up->down transition: Advance through the modes.
                    if (power_mode == 0) {
                        power_mode = 1;
                    } else if (power_mode == 1) {
                        power_mode = 2;
                        dim_mode = 0;
                        bat_counter = 0;
                    } else if (power_mode == 2) {
                        power_mode = 3;
                    } else if (power_mode == 3) {
                        power_mode = 0;
                    }
                    
                    counter = 0;
                    
                    button_down = 1;
                } else {
                    // If the button is held down for one second, toggle day/night.
                    if (counter > 100 && power_mode == 3) {
                        night_mode = !night_mode;
                        power_mode = 2;
                    }
                }
            } else {
                if (button_down) {
                    // Down-up transition.
                    button_down = 0;
                }
            }
            
            // Do the appropriate thing for the current mode.
            if (power_mode == 0) {
                // Off.
                LATCbits.LATC2 = 0;  // Red
                LATCbits.LATC3 = 0;  // Orange
                LATCbits.LATC4 = 0;  // Blue
                PWM1CONbits.PWM1EN = 0;
                PWM1CONbits.PWM1OE = 0;
                LATCbits.LATC0 = 0;  // Down
                LATCbits.LATC5 = 0;  // Front
            } else if (power_mode == 1) {
                // Ready to turn on (single press, waiting for a second press).
                LATCbits.LATC2 = 0;  // Red
                LATCbits.LATC3 = 0;  // Orange
                LATCbits.LATC4 = 1;  // Blue
                PWM1CONbits.PWM1EN = 0;
                PWM1CONbits.PWM1OE = 0;
                LATCbits.LATC0 = 0;  // Down
                LATCbits.LATC5 = 0;  // Front
                if (counter > 100) {
                    // Abandon waiting.
                    power_mode = 0;
                    counter = 0;
                }
            } else if (power_mode == 2) {
                // On mode.
                LATCbits.LATC2 = dim_mode;  // Red
                LATCbits.LATC3 = 1;  // Orange
                LATCbits.LATC4 = 0;  // Blue
                PWM1CONbits.PWM1EN = 1;
                PWM1CONbits.PWM1OE = 1;
                
                uint16_t b = 0;
                if (night_mode) {
                    if (dim_mode) {
                        // Night, low batt. forward-flash, down-flash, dim-forward.
                        if (counter % 83 < 7) {
                            LATCbits.LATC0 = 1;  // Down
                            b = PWM_PERIOD / 2;
                        } else if (counter % 83 < 14) {
                            LATCbits.LATC0 = 0;  // Down
                            b = PWM_PERIOD * 2;
                        } else {
                            LATCbits.LATC0 = 0;  // Down
                            b = PWM_PERIOD / 2;
                        }
                    } else {
                        // Night, normal. Down on, pulse front.
                        LATCbits.LATC0 = 1;  // Down
                        b = counter % (PWM_PERIOD * 3) + PWM_PERIOD;
                    }
                } else {
                    // Day mode. Forward double-flash only.
                    LATCbits.LATC0 = 0;  // Down
                    if (counter % 97 < 11) {
                        b = PWM_PERIOD;
                    } else if (counter % 97 < 16) {
                        b = 0;
                    } else if (counter % 97 < 27) {
                        b = PWM_PERIOD;
                    } else {
                        b = 0;
                    }
                }
                
                // Set PWM duty cycle based on brightness.
                PIR1bits.TMR2IF = 0;
                while (PIR1bits.TMR2IF == 0);
                PWM1DCLbits.PWM1DCL = b & 0x3;
                PWM1DCHbits.PWM1DCH = b >> 2;
                
                // Every second, measure the battery voltage.
                if (counter % 100 == 0) {
                    ADCON0bits.GO_nDONE = 1;
                    while (ADCON0bits.GO_nDONE == 1);
                    uint16_t vbat = ADRESH;
                    vbat <<= 8;
                    vbat |= ADRESL;
                    PIR1bits.ADIF = 0;
                    
                    if (vbat < 540) {
                        // Below 3.55V -- turn off.
                        power_mode = 0;
                    } else if (vbat < 566) {
                        // Below 3.65V. If we get 10 in a row, low power mode.
                        bat_counter++;
                        if (bat_counter > 10) {
                            // Latch dim_mode.
                            dim_mode = 1;
                        }
                    } else {
                        // Good voltage, reset counter.
                        bat_counter = 0;
                    }
                }
            } else if (power_mode == 3) {
                // Ready to turn off, waiting for second press.
                LATCbits.LATC2 = dim_mode;  // Red
                LATCbits.LATC3 = 0;  // Orange
                LATCbits.LATC4 = 1;  // Blue
                if (counter > 100) {
                    // Abandon waiting.
                    power_mode = 2;
                    counter = 0;
                }
            }
        }
        
        // Time forward by 10ms.
        counter += 1;
    }
    return;
}
