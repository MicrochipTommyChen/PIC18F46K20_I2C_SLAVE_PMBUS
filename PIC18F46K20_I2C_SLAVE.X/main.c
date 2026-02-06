/*
 * File:   main.c
 * Author: Tommy Chen
 *
 * Created on 20260205, 3:40
 */

#include <xc.h>
#include <stdint.h>

// -- Configuration Bits --
// MCU Configuration Settings
#pragma config FOSC = INTIO67   // Internal Oscillator Block
#pragma config WDTEN = OFF      // Watchdog Timer Disabled
#pragma config LVP = OFF        // Low Voltage Programming Disabled
#pragma config BOREN = OFF      // Brown-out Reset Disabled

#define _XTAL_FREQ 16000000     // Oscillator Frequency (Set to 16MHz)

// -- PMBus Definitions --
#define SLAVE_ADDRESS 0x50      // Default PMBus Slave Address

// -- PMBus Command Codes --
#define PMBUS_READ_VOUT   0x8B
#define PMBUS_READ_IOUT   0x8C
#define PMBUS_READ_TEMP1  0x8D
#define PMBUS_OPERATION   0x01

// -- I2C State Machine --
typedef enum {
    I2C_STATE_IDLE,
    I2C_STATE_ADDR_RECEIVED,
    I2C_STATE_DATA_RECEIVED,
    I2C_STATE_SEND_DATA
} i2c_state_t;

volatile i2c_state_t i2c_state = I2C_STATE_IDLE;
volatile uint8_t i2c_rx_buffer[8];
volatile uint8_t i2c_tx_buffer[8];
volatile uint8_t i2c_rx_index = 0;
volatile uint8_t i2c_tx_index = 0;
volatile uint8_t current_pmbus_command = 0;

// -- PMBus Data Simulation (Dummy Data) --
volatile uint16_t vout_reading = 0x1122; // Simulated VOUT (Linear-11 Format)
volatile uint16_t iout_reading = 0x0CCC; // Simulated IOUT (Linear-11 Format)
volatile uint16_t temp_reading = 0x012C; // Simulated TEMP (Linear-11 Format)
volatile uint8_t operation_status = 0x80; // Operation Status Byte

// -- Function Prototypes --
void SYSTEM_Initialize(void);
void I2C_Slave_Initialize(void);
void TIMER2_Initialize(void); // Initialize Timer2 for timing
void PMBUS_Process_Command(void);
void __interrupt() ISR(void);

void main(void) {
    SYSTEM_Initialize();
    I2C_Slave_Initialize();
    TIMER2_Initialize();    // Start Timer2

    // Global Interrupts are typically enabled here for I2C operation
     INTCONbits.PEIE = 1;
     INTCONbits.GIE = 1;

    while (1) {
        // Main loop is empty; processing is handled by I2C interrupts
    }
}

void SYSTEM_Initialize(void) {
    // Set Oscillator to 16MHz
    OSCCON = 0x70;
    OSCTUNE = 0x40;

    // Configure I/O pins
    ANSEL = 0x00;   // Digital I/O
    ANSELH = 0x00;  // Digital I/O

    // I2C Pin Configuration (Inputs)
    TRISCbits.TRISC3 = 1; // SCL
    TRISCbits.TRISC4 = 1; // SDA

    // Configure RD0 as Output (LED/Debug)
    TRISDbits.TRISD0 = 0;
    LATDbits.LATD0 = 0;   // Initialize Low
}

void I2C_Slave_Initialize(void) {
    // Initialize MSSP module as I2C Slave
    SSPSTAT = 0x80; // Slew Rate Control disabled for standard speed mode (100kHz)
    SSPCON1 = 0x36; // Enable Serial Port, I2C Slave mode, 7-bit address
//  SSPCON2 = 0x01; // Enable Clock Stretching (Optional)
    SSPCON2 = 0x00;
    SSPADD = SLAVE_ADDRESS << 1; // Set Slave Address (Shifted left by 1)

    // Interrupt Configuration
    PIR1bits.SSPIF = 0;      // Clear MSSP Interrupt Flag
    PIE1bits.SSPIE = 1;      // Enable MSSP Interrupt
    INTCONbits.PEIE = 1;     // Enable Peripheral Interrupts
    INTCONbits.GIE = 1;      // Enable Global Interrupts
}

void TIMER2_Initialize(void) {
    // Configure Timer2 for approx 100us Interrupt
    // Fosc = 16MHz, Fcy = Fosc/4 = 4MHz, Tcy = 0.25us
    // 100us / 0.25us = 400 cycles
    // Using 1:4 Prescaler: 400 / 4 = 100 cycles
    // PR2 = 100 - 1 = 99
    T2CON = 0x00;             // Postscaler 1:1, Timer2 off, Prescaler 1:1
    T2CONbits.T2CKPS = 0b01;  // Set Prescaler to 1:4
    PR2 = 99;                 // Set Period Register

    PIR1bits.TMR2IF = 0;      // Clear Timer2 Interrupt Flag
    PIE1bits.TMR2IE = 1;      // Enable Timer2 Interrupt

    T2CONbits.TMR2ON = 1;     // Turn on Timer2
}

void PMBUS_Process_Command(void) {
    current_pmbus_command = i2c_rx_buffer[0];

    switch (current_pmbus_command) {
        case PMBUS_READ_VOUT:
            i2c_tx_buffer[0] = vout_reading & 0xFF;
            i2c_tx_buffer[1] = (vout_reading >> 8) & 0xFF;
            i2c_tx_index = 2;
            break;

        case PMBUS_READ_IOUT:
            i2c_tx_buffer[0] = iout_reading & 0xFF;
            i2c_tx_buffer[1] = (iout_reading >> 8) & 0xFF;
            i2c_tx_index = 2;
            break;

        case PMBUS_READ_TEMP1:
            i2c_tx_buffer[0] = temp_reading & 0xFF;
            i2c_tx_buffer[1] = (temp_reading >> 8) & 0xFF;
            i2c_tx_index = 2;
            break;

        case PMBUS_OPERATION:
            if (i2c_rx_index > 1) { // Write Operation
                operation_status = i2c_rx_buffer[1];
            } else { // Read Operation
                i2c_tx_buffer[0] = operation_status;
                i2c_tx_index = 1;
            }
            break;

        default:
            // Unsupported command
            break;
    }
}

void __interrupt() ISR(void) {
    // Handle Timer2 Interrupt
    if (PIR1bits.TMR2IF && PIE1bits.TMR2IE) {
      LATDbits.LATD0 = ~LATDbits.LATD0; // Toggle RD0 LED (Debug)
        PIR1bits.TMR2IF = 0;              // Clear Timer2 Interrupt Flag
    }

//  // Handle I2C (MSSP) Interrupt
    if (PIR1bits.SSPIF && PIE1bits.SSPIE) {
        uint8_t temp_sspbuf;
//        LATDbits.LATD0 = ~LATDbits.LATD0; // Toggle RD0 LED to indicate activity

        // Case 1: Master Write, Address Matched
        if ((SSPSTATbits.S == 1) && (SSPSTATbits.D_A == 0) && (SSPSTATbits.R_W == 0)) {
            temp_sspbuf = SSPBUF; // Read SSPBUF to clear BF flag
            i2c_state = I2C_STATE_ADDR_RECEIVED;
            i2c_rx_index = 0;
        }
        // Case 2: Master Write, Data Received
        else if ((SSPSTATbits.S == 1) && (SSPSTATbits.D_A == 1) && (SSPSTATbits.R_W == 0)) {
            if (i2c_state == I2C_STATE_ADDR_RECEIVED || i2c_state == I2C_STATE_DATA_RECEIVED) {
                i2c_rx_buffer[i2c_rx_index++] = SSPBUF;
                i2c_state = I2C_STATE_DATA_RECEIVED;
            }
        }
        // Case 3: Master Read, Address Matched
        else if ((SSPSTATbits.S == 1) && (SSPSTATbits.D_A == 0) && (SSPSTATbits.R_W == 1)) {
            temp_sspbuf = SSPBUF; // Clear buffer
            PMBUS_Process_Command(); // Process the received command and prepare TX buffer
            i2c_tx_index = 0;
            SSPBUF = i2c_tx_buffer[i2c_tx_index++];
            i2c_state = I2C_STATE_SEND_DATA;
            SSPCON1bits.CKP = 1; // Release Clock
        }
        // Case 4: Master Read, Data Transmitted
        else if ((SSPSTATbits.S == 1) && (SSPSTATbits.D_A == 1) && (SSPSTATbits.R_W == 1)) {
            if (SSPCON1bits.WCOL == 0) { // Check for Write Collision
                SSPBUF = i2c_tx_buffer[i2c_tx_index++];
                SSPCON1bits.CKP = 1; // Release Clock
            }
        }
        // Case 5: Stop or Restart Condition detected
        else if ((SSPSTATbits.P == 1) || (SSPSTATbits.S == 1 && SSPSTATbits.D_A == 0)) {
             if (i2c_state == I2C_STATE_DATA_RECEIVED) {
                PMBUS_Process_Command(); // Execute command if needed
            }
            i2c_state = I2C_STATE_IDLE;
            i2c_rx_index = 0;
        }

        PIR1bits.SSPIF = 0; // Clear I2C Interrupt Flag
    }
}