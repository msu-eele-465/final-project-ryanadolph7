#include <msp430.h>
//#include "RGB_LED.h"
#include "intrinsics.h"
#include <math.h>

int upperBits[8];
int lowerBits[8];
int binTemp[16];

#define GYRO_I2C_ADDR      0x6A      // SA0 = 0 → 1101010₂
#define REG_CTRL2_G        0x11      // Gyro control register
#define REG_OUTX_L_G       0x22      // X-axis low byte register
#define GYRO_SENSITIVITY   (8.75f/1000.0f)  // ±245 dps → 8.75 mdps/LSB = 0.00875 dps/LSB

float wx, wy, wz;
float dx, dy, dz;
int gx, gy, gz;
int buf[6] = {0, 0, 0, 0, 0, 0};

int data_cnt = 0;
//char led_packet[] = {0x69, '1'};
char led_packet[] = {'1','4','4',};
//               slave addr, data to send
int rx_count = 0;
#define GYRO_REG_START 0x20
#define GYRO_RX_LEN    10

unsigned char tx_data[4] = { '1', '3', 0xD4 }; // '1', '3', 0x00D4 = 212 (21.2°C)
unsigned char tx_index = 0;

char received[16];

char gyro_received[10];
// --------------------------------------------------------------------------------
// I2C init
void initI2C() {
// 1 - Put eUSCI_B0 into software reset
   UCB0CTLW0 |= UCSWRST;           // UCSWRST = 1 for eUSCI_B0 in SW reset

   // 2 - Configure eUSCI_B0
   UCB0CTLW0 |= UCSSEL__SMCLK;     // chose BRCLK = SMCLK = 1 MHz
   UCB0BRW = 10;                   // divide BRCLK by 10 for SCL = 100 kHz

   UCB0CTLW0 |= UCMODE_3;          // put into I2C mode
   UCB0CTLW0 |= UCMST;             // put into master mode
   UCB0CTLW0 &= ~UCTR;              // put into Tx mode
   //UCB0CTLW0 |= UCSYNC;
   UCB0I2CSA = 0x6A;              // slave address = 0x6A (Gryo slave initially)

   UCB0CTLW1 |= UCASTP_2;          // auto STOP when UCB0TBCNT reached
   UCB0TBCNT = sizeof(gyro_received);     // # of bytes in packet

   // 3 - configure ports
   P1SEL1 &= ~BIT3;
   P1SEL0 |= BIT3;                 // want P1.3 = SCL

   P1SEL1 &= ~BIT2;
   P1SEL0 |= BIT2;                 // want P1.2 = SDA

   // 4 - take eUSCI_B0 out of SW reset
   UCB0CTLW0 &= ~UCSWRST;          // UCSWRST = 1 for eUSCI_B0 in SW reset

   // 5 - enable interrupts
   UCB0IE |= UCTXIE0;              // enable I2C Tx0 IRQ
   UCB0IE |= UCRXIE0;              // enable I2C Rx0 IRQ
   UCB0CTLW0 |= UCTXSTT;
   __enable_interrupt();           // enable global interrupts
}
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// Gyro code 
void initial_gryo_write(int addr, int reg, int data) {
    UCB0I2CSA  = addr;
    UCB0CTLW0 |= UCTR | UCTXSTT;                   // TX, START
    while (!(UCB0IFG & UCTXIFG0));
    UCB0TXBUF  = reg;
    while (!(UCB0IFG & UCTXIFG0));
    UCB0TXBUF  = data;
    while (!(UCB0IFG & UCTXIFG0));
    UCB0CTLW0 |= UCTXSTP;                          // STOP
    __delay_cycles(100000);
}

void gyro_read_raw(int *gx, int *gy, int *gz) {
    //int buf[6];
    i2c_read(GYRO_I2C_ADDR, REG_OUTX_L_G, buf, 6);
    *gx = (int)((gyro_received[1] << 8) | gyro_received[0]);
    *gy = (int)((gyro_received[3] << 8) | gyro_received[2]);
    *gz = (int)((gyro_received[5] << 8) | gyro_received[4]);
}

void gyro_read_dps(float dx, float dy, float dz) {
    //int gx, gy, gz;
    gyro_read_raw(&gx, &gy, &gz);
    dx = gx * GYRO_SENSITIVITY;
    dy = gy * GYRO_SENSITIVITY;
    dz = gz * GYRO_SENSITIVITY;
}

void i2c_read(int addr, int reg, int buf[], int len) {
    UCB0I2CSA  = 0x6A;
    UCB0CTLW0 |= UCTR | UCTXSTT;                   // TX mode, START
    //while (!(UCB0IFG & UCTXIFG0));                 // wait TX ready
    //__delay_cycles(10000);
    UCB0TXBUF = reg | 0x80;                        // auto-increment
    //while (!(UCB0IFG & UCTXIFG0));
    __delay_cycles(10000);
    
    UCB0CTLW0 &= ~UCTR;                            // RX mode
    UCB0CTLW0 |= UCTXSTT;                          // Repeated START
    int i;
    for (i = 0; i < len; i++) {
        //while (!(UCB0IFG & UCRXIFG0));             // wait RX ready
        //__delay_cycles(100000);
        if (i == len - 1) {
            UCB0CTLW0 |= UCTXSTP;                  // STOP before last byte
        }
        buf[i] = UCB0RXBUF;                        // read byte
    }
    /*
    UCB0CTLW0 |= UCTR | UCTXSTT;                   // TX, START
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    while (!(UCB0IFG & UCTXIFG0));
    UCB0TXBUF  = reg | 0x80;                       // Auto-increment
    while (!(UCB0IFG & UCTXIFG0));
    UCB0CTLW0 &= ~UCTR;                            // RX
    UCB0CTLW0 |= UCTXSTT;                          // Repeated START
    int i;
    for (i = 0; i < len; i++) {
        while (!(UCB0IFG & UCRXIFG0));
        buf[i] = UCB0RXBUF;
        if (i == len - 1) UCB0CTLW0 |= UCTXSTP;    // STOP on last byte
    }
    */
    __delay_cycles(10);
}

// -------------------------------------------------------------------------------

void sendI2C(void) {
    UCB0CTLW0 |= UCTR;  // transmitter 
    UCB0CTLW0 |= UCTXSTT;   // start condition
    while ((!UCB0IFG & UCTXIFG0));

}

void startMSPRead(void) {

}

/*
void startGyroRead(void) {
    rx_count = 0;

    // 1) TX the register pointer
    UCB0TBCNT = GYRO_RX_LEN;
    //UCB0CTLW0 |= UCTR;    // transmitter
    UCB0CTLW0 &= ~UCTR;   // receiver
    UCB0CTLW0 |= UCTXSTT; // START
    while ((!UCB0IFG & UCTXIFG0));
    UCB0TXBUF = GYRO_REG_START;
    //UCB0CTLW0 &= ~UCTR;   // receiver
    while ((!UCB0IFG & UCTXIFG0));

    // 2) Repeated-START for read
    UCB0CTLW0 &= ~UCTR;   // receiver
    UCB0CTLW0 |= UCTXSTT; // repeated START
    // hardware will clock in bytes and the ISR will fire
}
*/ 

void timer_init(void) {
    TB0CCR0 = 16384 - 1;                      // Period = 16384 counts (~500ms)
    TB0CCTL0 = CCIE;                          // Enable CCR0 interrupt
    TB0CTL = TBSSEL__ACLK | MC__UP | TBCLR;     // Use ACLK, up mode, clear timer
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer_B_ISR(void) {
    P1OUT ^= BIT1;  // Toggle heartbeat LED on P1.1
    //rising_edge = TB1CCR1;
    TB0CCTL0 &= ~CCIFG;
}

int main(void)
{
    // Stop watchdog timer
   
    WDTCTL = WDTPW | WDTHOLD;
   // Configure heartbeat LED on P1.1 as output.
    P1DIR |= BIT1;
    P1OUT &= ~BIT1;
    int cfg2 = (0x4 << 4) | (0x0 << 2);     
    initial_gryo_write(GYRO_I2C_ADDR, REG_CTRL2_G, cfg2);
    initI2C();
    timer_init();

    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                           // to activate previously configured port settings
    __enable_interrupt();
   // Main loop enters low-power mode; all processing is in the ISR.
    int i;
   while(1){
       int j;
        //startGyroRead();        // read from gyro
        gyro_read_dps(wx, wy, wz);
        //__delay_cycles(100000);  // ~100 ms at 1 MHz
        //startMSPRead();         // read from 2310 
        //__delay_cycles(100000);  // ~100 ms at 1 MHz
        //sendI2C();              // send to LCD
        //__delay_cycles(100000);  // ~100 ms at 1 MHz

        // process gyro_buf[]
    
            //__delay_cycles(5000);
            //UCB0CTLW0 &= ~UCTR;			// put into Rx mode
            //UCB0I2CSA = 0x6A;             // slave address = 0x69
            //UCB0TBCNT = sizeof(gyro_received);     // # of bytes in packet
            //UCB0CTLW0 |= UCTXSTT;       // generate start condition
            //while (UCB0CTLW0 & UCTXSTP);  // Wait for stop condition
            //UCB0IFG &= ~UCSTPIFG;   // clear STOP flag
            //portflag = 0;

           // __delay_cycles(5000);

       //}
       
   }
}





// update this to receive from 2310 
// and to send to lcd
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch (UCB0IV) {
    case USCI_I2C_UCRXIFG0:
        gyro_received[rx_count++] = UCB0RXBUF;
        if (rx_count == GYRO_RX_LEN - 1) {
            UCB0CTLW0 |= UCTXSTP;  // STOP after last byte
        }
        break;

    case USCI_I2C_UCSTPIFG:
        // STOP detected: gyro_buf[] is ready
        break;

    default:
        break;
    }
}


// I2C interrupt for sending a single packet to the LED bar
//#pragma vector=EUSCI_B0_VECTOR
/*__interrupt void EUSCI_B0_I2C_ISR(void) {
   int j;


    if(UCB0I2CSA == 0x48){
        if(data_cnt == (sizeof(tx_data) - 1)) { // transmit data 
            UCB0TXBUF = tx_data[data_cnt];
            data_cnt = 0;
        } else {
            UCB0TXBUF = tx_data[data_cnt];
            data_cnt++;
        }
    }else if(UCB0I2CSA == 0x69){        // transmit led_packet
        UCB0TXBUF = led_packet[0];      
    } else if (UCB0IV == 0x16) { // receive data
        //if (UCB)
        data_cnt = 15;
        for (data_cnt; data_cnt >= 0; data_cnt--) {
            received[data_cnt] = UCB0RXBUF;
        }
    }  else if (UCB0IV == 0x6A) {       // receive from gyro
        data_cnt = 0;
        for(data_cnt; data_cnt < 10; data_cnt++) {
            gyro_received[data_cnt] = UCB0RXBUF;
        }
    }
}
*/