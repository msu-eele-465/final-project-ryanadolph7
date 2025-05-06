#include "intrinsics.h"
#include <msp430.h>
#include <string.h>
#include <math.h>

// Define bit masks for LED and LCD pins
#define LED         BIT1          // Heartbeat LED on P1.1

// LCD data lines on P1.4, P1.5, P1.6, and P1.7
#define LCD_D4      BIT4
#define LCD_D5      BIT5
#define LCD_D6      BIT6
#define LCD_D7      BIT7
#define LCD_DATA    (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7)

// LCD control pins on P2.6 (RS) and P2.7 (Enable)
#define LCD_RS      BIT6          // P2.6
#define LCD_E       BIT7          // P2.7

// Global variables for I2C input

volatile char button = 'B';
volatile char window = '3';
volatile char temp[8] = "21.2";
volatile int temp_raw = 212; // Raw temperature in tenths of °C
volatile unsigned char rx_index = 0;
volatile unsigned char rx_buffer[3]; // Holds button, window, temp_high, temp_low
volatile int  type_select = 0;         // Holds value to select type of screen output.
volatile unsigned char newButtonFlag = 0; // Flag set when new button received

// variable for sensor input

volatile int rising_edge, falling_edge, pulse_width;
volatile int  edge = 0; 
volatile int distance = 0; 

//volatile unsigned int rising_edge = 0;
//volatile unsigned int falling_edge = 0;
//volatile unsigned int prev_rising_edge = 0;
//volatile float duty_cycle = 0;



//-------------------------------------------------
// Timer Initialization for Heartbeat LED & Delay
//-------------------------------------------------
void timer_init(void) {
    TB0CCR0 = 16384 - 1;                      // Period = 16384 counts (~500ms)
    TB0CCTL0 = CCIE;                          // Enable CCR0 interrupt
    TB0CTL = TBSSEL__ACLK | MC__UP | TBCLR;     // Use ACLK, up mode, clear timer
}

// need to reconfig to send instead of receive
void i2c_init(void) {
    UCB0CTLW0 = UCSWRST;                      // Put eUSCI_B0 in reset

    // Configure P1.2 and P1.3 for I2C mode
    P1SEL1 &= ~(BIT3 | BIT2);
    P1SEL0 |= (BIT3 | BIT2);
    
    UCB0CTLW0 |= UCTR;                       // transmitter mode
    UCB0CTLW0 &= ~UCMST;                       // Slave mode
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;            // I2C, synchronous mode
    UCB0I2COA0 = 0x48 | UCOAEN;                // Set own address to 0x48 and enable it
    UCB0I2COA0 |= UCGCEN;                      // Enable general call response

    // Enable I2C receive and start condition interrupts
    UCB0IE |= (UCRXIE | UCSTTIE);

    UCB0CTLW0 &= ~UCSWRST;                     // Release from reset
    
    // Enable I2C receive and start condition interrupts
    
}


//-------------------------------------------------
// Timer_B CCR0 Interrupt Service Routine
//-------------------------------------------------
#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer_B_ISR(void) {
    P1OUT ^= LED;  // Toggle heartbeat LED on P1.1
    //rising_edge = TB1CCR1;
    TB0CCTL0 &= ~CCIFG;
}


// Timer1_B1 ISR
#pragma vector = TIMER1_B1_VECTOR
__interrupt void TIMER1_B1_ISR(void)
{
    switch (__even_in_range(TB1IV, TB1IV_TBIFG)) {
      case TB1IV_NONE:    // no interrupt pending
        break;

      case TB1IV_TBCCR1:  // CCR1 capture event
        if (edge == 0) {
          rising_edge = TB1CCR1;
          // now switch to falling-edge capture
          TB1CCTL1 = CM_2 | CCIS_0 | SCS | CAP | CCIE;
          edge = 1;
        } else {
          falling_edge = TB1CCR1;
          // handle timer wrap-around
          pulse_width = (falling_edge >= rising_edge)
                        ? (falling_edge - rising_edge)
                        : (0xFFFF - rising_edge + falling_edge);
          // switch back to rising-edge capture
          TB1CCTL1 = CM_1 | CCIS_0 | SCS | CAP | CCIE;
          edge = 0;
        }
        distance = 2 * (pulse_width - 1000); // should be in mm
        distance /= 10;                     // converts to cm
        break;

      default:
        break;
    }
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer
    
    
    // Configure heartbeat LED on P1.1 as output.
    P1DIR |= LED;
    P1OUT &= ~LED;
    
    // Configure LCD data pins (P1.4–P1.7) as outputs.
    P1DIR |= LCD_DATA;
    P1OUT &= ~LCD_DATA;
    
    // Configure LCD control pins (P2.6 and P2.7) as outputs.
    P2DIR |= (LCD_RS | LCD_E);
    P2OUT &= ~(LCD_RS | LCD_E);

    // configure P2.0 as input for PWM for PW Distance Sensor

    P2DIR &= ~BIT0;        // P2.0 as input
    P2SEL0 |= BIT0;        // Select TB1.1 function
    P2SEL1 &= ~BIT0;

    TB1CCTL1 = CM_1 | CCIS_0 | SCS | CAP | CCIE;         // Capture rising edge, CCIxA, sync, capture mode, interrupt
    TB1CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR;     // SMCLK, continuous mode, clear timer
    
    timer_init();   // Initialize Timer_B for delays and LED heartbeat.

    PM5CTL0 &= ~LOCKLPM5;  // Unlock GPIO
    __bis_SR_register(GIE); // Enable global interrupts
    
    i2c_init();     // Initialize I2C in slave mode.
    //lcd_init();     // Initialize the LCD.
    
    //update_display(); // Initial display update.
    
  
    while(1) {

        int t;
        

        /*update_display();  // Update display based on new I2C input.
        __delay_cycles(500000);
        __delay_cycles(500000);
        __delay_cycles(500000);
        button = 'A';
        window = '5';
        update_display();
        __delay_cycles(500000);
        __delay_cycles(500000);
        window = '6';
        update_display();
        __delay_cycles(500000);
        __delay_cycles(500000);
        button = 'B';
        window = '6';
        update_display();
        __delay_cycles(500000);
        __delay_cycles(500000);
        window = '9';
        update_display();
        __delay_cycles(500000);
        button = 'D';
        update_display();
        button = '3';
        __delay_cycles(500000);
        __delay_cycles(500000);
        __delay_cycles(500000);
        __delay_cycles(500000);

        */

        /*switch(button) {
        case '9': button = 'A'; break;
        case 'A': button = 'B'; break;
        case 'B': button = 'C'; break;
        case 'C': button = 'D'; break;      // demo code incase nothing works properly
        case 'D': button = '*'; break;
        case '*': button = '#'; break;
        case '#': button = '0'; break;
        default:  button++; break;*/ 
    
    }
    
    return 0;
}
