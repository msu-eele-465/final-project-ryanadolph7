#include "intrinsics.h"
#include "msp430fr2355.h"
#include <msp430.h>
#include <stdint.h>
#include "RGB_LED.h"

#define PWM_MAX 255  // PWM period (8-bit resolution)
#define PWM_PIN   BIT0  // Port2.0



// Initialize PWM for the RGB LED.
// Uses Timer_B2 for red and green channels and Timer_B3 for the blue channel.
void initPWM(void)
{
    // --- Configure LED pins as outputs ---
    // Red and Green on Port2 (Timer_B2 channels)
    P2DIR |= PWM_PIN;
    P2OUT &= ~PWM_PIN;
    
    // --- Timer_B2 Setup for Red and Green PWM ---
    TB2CCR0 = PWM_MAX;
    TB2CCR1 = 0; // Red initially off
    TB2CCR2 = 0; // Green initially off
    TB2CCTL0 = CCIE;
    TB2CCTL1 = CCIE;
    TB2CCTL2 = CCIE;
    TB2CTL = TBSSEL__SMCLK | MC__UP | TBCLR;
    
    // --- Timer_B3 Setup for Blue PWM ---
    TB3CCR0 = PWM_MAX;
    TB3CCR1 = 0;  // Blue initially off.
    TB3CCTL0 = CCIE;
    TB3CCTL1 = CCIE;
    TB3CTL = TBSSEL__SMCLK | MC__UP | TBCLR;
    
    __enable_interrupt();
}

// --- Timer_B2 ISR for red and green PWM ---
// CCR0: start of PWM period for red and green.
#pragma vector = TIMER2_B0_VECTOR
__interrupt void TB2_CCR0_ISR(void)
{
    P2OUT |= PWM_PIN;
}

// ISR for CCR1 (red) and CCR2 (green) using TIMER2_B1_VECTOR.
#pragma vector = TIMER2_B1_VECTOR
__interrupt void TB2_B1_ISR(void)
{
    switch (__even_in_range(TB2IV, 4))
    {
        case 0: break;
        case 2:
            P2OUT &= ~PWM_PIN;
            break;
        case 4:
            //P2OUT &= ~GREEN_PIN;
            break;
        default: break;
    }
}

// --- Timer_B3 ISR for blue PWM ---
// CCR0: start of PWM period for blue.
#pragma vector = TIMER3_B0_VECTOR
__interrupt void TB3_CCR0_ISR(void)
{
    //P4OUT |= BLUE_PIN;
}

// ISR for CCR1 (blue) using TIMER3_B1_VECTOR.
#pragma vector = TIMER3_B1_VECTOR
__interrupt void TB3_B1_ISR(void)
{
    switch (__even_in_range(TB3IV, 2))
    {
        case 0: break;
        case 2:
            //P4OUT &= ~BLUE_PIN;
            break;
        default: break;
    }
}
