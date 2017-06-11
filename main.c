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
// RA4 -- Battery sense (Vbatt/2)
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
    // high-z, analog batt sense
    TRISAbits.TRISA4 = 1;
    ANSELAbits.ANSA4 = 1;
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
    
    // Power on / reset indication.
    LATCbits.LATC1 = 1;
    LATCbits.LATC2 = 1;
    LATCbits.LATC3 = 1;
    LATCbits.LATC4 = 1;
    __delay_ms(200);
    LATCbits.LATC1 = 0;
    LATCbits.LATC2 = 0;
    LATCbits.LATC3 = 0;
    LATCbits.LATC4 = 0;
    
    // Front LED PWM
    PWM1CONbits.PWM1EN = 1;
    PWM1CONbits.PWM1OE = 1;
    PWM1DCLbits.PWM1DCL = 0;
    PWM1DCHbits.PWM1DCH = 0;
    // Reset Timer2
    T2CON = 0;
    PR2bits.PR2 = PWM_PERIOD - 1;
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11; // 64:1
    T2CONbits.TMR2ON = 1;
    while (PIR1bits.TMR2IF == 0);
    
    uint16_t counter = 0;
    uint8_t button_down = 0;
    uint8_t power_mode = 0;
    
    while (1) {
        __delay_ms(10);
        CLRWDT();
        
        if (PORTAbits.RA5 == 0) {
            // Charging - turn off lights and flash Green LED at 0.5 Hz.
            power_mode = 0;
            
            PWM1CONbits.PWM1EN = 0;
            PWM1CONbits.PWM1OE = 0;
            
            LATCbits.LATC0 = 0;  // Down
            LATCbits.LATC3 = 0;  // Orange
            LATCbits.LATC4 = 0;  // Blue
            LATCbits.LATC5 = 0;  // Front
            
            if (counter > 200) {
                LATCbits.LATC1 = 0;  // Green
                counter = 0;
            } else if (counter > 100) {
                LATCbits.LATC1 = 1;  // Green
            }
        } else {
            // Not charging.
            LATCbits.LATC1 = 0;  // Green
            
            // Detect off->on button sequences.
            if (PORTAbits.RA2 == 0) {
                if (!button_down) {
                    // Advance through the modes.
                    if (power_mode == 0) {
                        power_mode = 1;
                    } else if (power_mode == 1) {
                        power_mode = 2;
                        PWM1CONbits.PWM1EN = 1;
                        PWM1CONbits.PWM1OE = 1;
                        LATCbits.LATC0 = 1;  // Down
                    } else if (power_mode == 2) {
                        power_mode = 3;
                    } else if (power_mode == 3) {
                        power_mode = 0;
                        PWM1CONbits.PWM1EN = 0;
                        PWM1CONbits.PWM1OE = 0;
                        LATCbits.LATC0 = 0;  // Down
                        LATCbits.LATC5 = 0;  // Front
                    }
                    
                    counter = 0;
                }
                
                button_down = 1;
            } else {
                button_down = 0;
            }
            
            if (power_mode == 0) {
                LATCbits.LATC3 = 0;  // Orange
                LATCbits.LATC4 = 0;  // Blue
            } else if (power_mode == 1) {
                LATCbits.LATC3 = 0;  // Orange
                LATCbits.LATC4 = 1;  // Blue
                if (counter > 100) {
                    power_mode = 0;
                    counter = 0;
                }
            } else if (power_mode == 2) {
                LATCbits.LATC3 = 1;  // Orange
                LATCbits.LATC4 = 0;  // Blue
                uint16_t b = counter % (PWM_PERIOD * 3) + PWM_PERIOD;
                PIR1bits.TMR2IF = 0;
                while (PIR1bits.TMR2IF == 0);
                PWM1DCLbits.PWM1DCL = b & 0x3;
                PWM1DCHbits.PWM1DCH = b >> 2;
            } else if (power_mode == 3) {
                LATCbits.LATC3 = 0;  // Orange
                LATCbits.LATC4 = 1;  // Blue
                if (counter > 100) {
                    power_mode = 2;
                    counter = 0;
                }
            }
        }
        
        counter += 1;
    }
    return;
}
