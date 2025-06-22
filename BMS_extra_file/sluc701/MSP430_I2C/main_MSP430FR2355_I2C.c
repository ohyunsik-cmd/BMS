/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//******************************************************************************
//   BQ76952EVM demo code for MSP430FR2x55 + BQ769x2
//
//   Description: MSP430FR2x55 functions as the I2C host that communicates
//   with the BQ769x2 sending and receiving different types of commands. 
//   ACLK = 32.768kHz REFO or external XT1, MCLK = SMCLK = ~16MHz DCO
//
//                                     /|\ /|\
//                    MSP430FR2x55     10k  |
//                 -----------------    |  10k
//            /|\ |             P4.6|---+---|-- I2C Data (UCB1SDA)
//             |  |                 |       |
//             ---|RST          P4.7|-------+-- I2C Clock (UCB1SCL)
//                |                 |
//                |             P2.3|---> SHUT
//                |                 |
//                |             P4.2|---> FETOFF
//                |                 |
//       LED1 <---|P1.1         P4.3|<--- ALERT
//                |                 |
//
//
//   Andrew Han and Matt Sunna
//   Texas Instruments Inc.
//   August 2021
//   Built with Code Composer Studio (CCS) v10.1.1
//******************************************************************************

// Includes
#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <Include/HAL.h>
#include <Include/QmathLib.h>
#include <Include/callbacks_mpack.h>
#include "BQ769x2Header.h"

// Defines
#define DEV_ADDR  0x08  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 0  // 0 for disabled, 1 for enabled
#define MAX_BUFFER_SIZE         10
#define R 0 //Read
#define W 1 //Write
#define W2 2 //write data with two bytes


// Arrays
uint8_t RX_data [2] = {0x00, 0x00};// used for DirectCommand func, 
uint8_t RX_32Byte [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};



//******************************************************************************
// I2C State Machine ***********************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;

/* Register address/command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

//******************************************************************************
// BQ Parameters ***************************************************************
//******************************************************************************

// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage [16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
float Temperature [3] = {0,0,0};
uint16_t Stack_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
uint16_t Pack_Current = 0x00;
uint16_t AlarmBits = 0x00;

uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;   // Permanent Fail Status Register A
uint8_t value_PFStatusB;   // Permanent Fail Status Register B
uint8_t value_PFStatusC;   // Permanent Fail Status Register C
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
uint16_t CB_ActiveCells;  // Cell Balancing Active Cells

uint8_t UV_Fault = 0;   // under-voltage fault state
uint8_t OV_Fault = 0;   // over-voltage fault state
uint8_t SCD_Fault = 0;  // short-circuit fault state
uint8_t OCD_Fault = 0;  // over-current fault state
uint8_t ProtectionsTriggered = 0; // Set to 1 if any protection triggers

uint8_t LD_ON = 0;                          // Load Detect status bit
uint8_t DSG = 0;   // discharge FET state
uint8_t CHG = 0;   // charge FET state
uint8_t PCHG = 0;  // pre-charge FET state
uint8_t PDSG = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int; // in AFE_READPASSQ func
uint32_t AccumulatedCharge_Frac;// in AFE_READPASSQ func
uint32_t AccumulatedCharge_Time;// in AFE_READPASSQ func


//******************************************************************************
// Function prototypes *********************************************************
//******************************************************************************

void GPIO_initPins(void);
void GPIO_configPins(void);
void UCS_initModule(void);
void I2C_initModule(void);
void UART_initPins(void);
void Software_Trim(void);

void delayUS(uint16_t us) {   // Sets the delay in microseconds.
    uint16_t ms;
    char i;
    ms = us / 1000;
    for(i=0; i< ms; i++)
        __delay_cycles(16000);
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
    unsigned char i;
    unsigned char checksum = 0;

    for(i=0; i<len; i++)
        checksum += ptr[i];

    checksum = 0xff & ~checksum;

    return(checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions
{
    unsigned char i;
    unsigned char crc=0;
    while(len--!=0)
    {
        for(i=0x80; i!=0; i/=2)
        {
            if((crc & 0x80) != 0)
            {
                crc *= 2;
                crc ^= 0x107;
            }
            else
                crc *= 2;

            if((*ptr & i)!=0)
                crc ^= 0x107;
        }
        ptr++;
    }
    return(crc);
}

I2C_Mode I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize device address and interrupts */
    UCB1I2CSA = DEV_ADDR;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);        // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                      // Disable RX interrupt
    UCB1IE |= UCTXIE;                       // Enable TX interrupt
    UCB1CTLW0 |= UCTR + UCTXSTT;            // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
    //For debugger
    __no_operation();

    /* Copy over received data */
    CopyArray(ReceiveBuffer, reg_data, count);

    return MasterMode;
}

I2C_Mode I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    /* Copy register data to TransmitBuffer */
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize device address and interrupts */
    UCB1I2CSA = DEV_ADDR;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);        // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                      // Disable RX interrupt
    UCB1IE |= UCTXIE;                       // Enable TX interrupt
    UCB1CTLW0 |= UCTR + UCTXSTT;            // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
    //For debugger
    __no_operation();

    return MasterMode;
}

void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
    uint8_t TX_Buffer[2] = {0x00, 0x00};
    uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    //TX_RegData in little endian format
    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff; //1st byte of data

    switch(datalen)
    {
        case 1: //1 byte datalength
            I2C_WriteReg(0x3E, TX_RegData, 3);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            delayUS(2000);
            break;
        case 2: //2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 4);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] = 0x06; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            delayUS(2000);
            break;
        case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 6);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] = 0x08; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            delayUS(2000);
            break;
    }
}

void CommandSubcommands(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{   //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    uint8_t TX_Reg[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    I2C_WriteReg(0x3E,TX_Reg,2);
    delayUS(2000);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if (type == R) {//read
        I2C_WriteReg(0x3E,TX_Reg,2);
        delayUS(2000);
        I2C_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
    }
    else if (type == W) {
        //FET_Control, REG12_Control
        TX_Reg[2] = data & 0xff;
        I2C_WriteReg(0x3E,TX_Reg,3);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05; //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
    else if (type == W2){ //write data with 2 bytes
        //CB_Active_Cells, CB_SET_LVL
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        I2C_WriteReg(0x3E,TX_Reg,4);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06; //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{   //type: R = read, W = write
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R) {//Read
        I2C_ReadReg(command, RX_data, 2); //RX_data is a global variable
        delayUS(2000);
    }
    if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
        I2C_WriteReg(command,TX_data,2);
        delayUS(2000);
    }
}

void BQ769x2_Init() {
    // Configures all parameters in device RAM

    // Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
    // See TRM for full description of CONFIG_UPDATE mode
    CommandSubcommands(SET_CFGUPDATE);

    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    BQ769x2_SetRegister(PowerConfig, 0x2D80, 2);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    BQ769x2_SetRegister(REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    BQ769x2_SetRegister(REG12Config, 0x0D, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    BQ769x2_SetRegister(DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    BQ769x2_SetRegister(TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    BQ769x2_SetRegister(TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    BQ769x2_SetRegister(VCellMode, 0x0000, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
    BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    BQ769x2_SetRegister(CUVThreshold, 0x31, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    BQ769x2_SetRegister(COVThreshold, 0x55, 1);

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    BQ769x2_SetRegister(OCCThreshold, 0x05, 1);

    // Set up OCD1 Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    BQ769x2_SetRegister(OCD1Threshold, 0x0A, 1);

    // Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    BQ769x2_SetRegister(SCDThreshold, 0x05, 1);

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 µs; min value of 1
    BQ769x2_SetRegister(SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1);

    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    CommandSubcommands(EXIT_CFGUPDATE);
}

//  ********************************* FET Control Commands  ***************************************

void BQ769x2_BOTHOFF () {
    // Disables all FETs using the DFETOFF (BOTHOFF) pin
    // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
    P4OUT |= BIT2;              // DFETOFF pin (BOTHOFF) set high
}

void BQ769x2_RESET_BOTHOFF () {
    // Resets DFETOFF (BOTHOFF) pin
    // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
    P4OUT &= ~BIT2;             // DFETOFF pin (BOTHOFF) set low
}

void BQ769x2_ReadFETStatus() {
    // Read FET Status to see which FETs are enabled
    DirectCommands(FETStatus, 0x00, R);
    FET_Status = (RX_data[1]*256 + RX_data[0]);
    DSG = ((0x4 & RX_data[0])>>2);// discharge FET state
    CHG = (0x1 & RX_data[0]);// charge FET state
    PCHG = ((0x2 & RX_data[0])>>1);// pre-charge FET state
    PDSG = ((0x8 & RX_data[0])>>3);// pre-discharge FET state
}

// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************

void BQ769x2_ShutdownPin() {
    // Puts the device into SHUTDOWN mode using the RST_SHUT pin
    // The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
    P2OUT |= BIT3;              // Set RST_SHUT pin
}

void BQ769x2_ReleaseShutdownPin() {
    // Releases the RST_SHUT pin
    // The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
    P2OUT &= ~BIT3;             // Reset RST_SHUT pin
}

// ********************************* End of BQ769x2 Power Commands   *****************************************


// ********************************* BQ769x2 Status and Fault Commands   *****************************************

uint16_t BQ769x2_ReadAlarmStatus() {
    // Read this register to find out why the ALERT pin was asserted
    DirectCommands(AlarmStatus, 0x00, R);
    return (RX_data[1]*256 + RX_data[0]);
}

void BQ769x2_ReadSafetyStatus() { //good example functions
    // Read Safety Status A/B/C and find which bits are set
    // This shows which primary protections have been triggered
    DirectCommands(SafetyStatusA, 0x00, R);
    value_SafetyStatusA = (RX_data[1]*256 + RX_data[0]);
    //Example Fault Flags
    UV_Fault = ((0x4 & RX_data[0])>>2);
    OV_Fault = ((0x8 & RX_data[0])>>3);
    SCD_Fault = ((0x8 & RX_data[1])>>3);
    OCD_Fault = ((0x2 & RX_data[1])>>1);
    DirectCommands(SafetyStatusB, 0x00, R);
    value_SafetyStatusB = (RX_data[1]*256 + RX_data[0]);
    DirectCommands(SafetyStatusC, 0x00, R);
    value_SafetyStatusC = (RX_data[1]*256 + RX_data[0]);
    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) > 1) {
        ProtectionsTriggered = 1; }
    else {
        ProtectionsTriggered = 0; }
}

void BQ769x2_ReadPFStatus() {
    // Read Permanent Fail Status A/B/C and find which bits are set
    // This shows which permanent failures have been triggered
    DirectCommands(PFStatusA, 0x00, R);
    value_PFStatusA = (RX_data[1]*256 + RX_data[0]);
    DirectCommands(PFStatusB, 0x00, R);
    value_PFStatusB = (RX_data[1]*256 + RX_data[0]);
    DirectCommands(PFStatusC, 0x00, R);
    value_PFStatusC = (RX_data[1]*256 + RX_data[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************


// ********************************* BQ769x2 Measurement Commands   *****************************************


uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
    //RX_data is global var
    DirectCommands(command, 0x00, R);
    if(command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
        return (RX_data[1]*256 + RX_data[0]); //voltage is reported in mV
    }
    else {//stack, Pack, LD
        return 10 * (RX_data[1]*256 + RX_data[0]); //voltage is reported in 0.01V units
    }

}
void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
  unsigned char x;
  int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
  for (x = 0; x < 16; x++){//Reads all cell voltages
    CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
    cellvoltageholder = cellvoltageholder + 2;
  }
  Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
  Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
  LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}

uint16_t BQ769x2_ReadCurrent()
// Reads PACK current
{
    DirectCommands(CC2Current, 0x00, R);
    return (RX_data[1]*256 + RX_data[0]);  // current is reported in mA
}

float BQ769x2_ReadTemperature(uint8_t command)
{
    DirectCommands(command, 0x00, R);
    //RX_data is a global var
    return (0.1 * (float)(RX_data[1]*256 + RX_data[0])) - 273.15;  // converts from 0.1K to Celcius
}

void BQ769x2_ReadPassQ(){ // Read Accumulated Charge and Time from DASTATUS6
    Subcommands(DASTATUS6, 0x00, R);
    AccumulatedCharge_Int = ((RX_32Byte[3]<<24) + (RX_32Byte[2]<<16) + (RX_32Byte[1]<<8) + RX_32Byte[0]); //Bytes 0-3
    AccumulatedCharge_Frac = ((RX_32Byte[7]<<24) + (RX_32Byte[6]<<16) + (RX_32Byte[5]<<8) + RX_32Byte[4]); //Bytes 4-7
    AccumulatedCharge_Time = ((RX_32Byte[11]<<24) + (RX_32Byte[10]<<16) + (RX_32Byte[9]<<8) + RX_32Byte[8]); //Bytes 8-11
}


//******************************************************************************
// main ************************************************************************
//******************************************************************************
int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Initialize GPIOs, clocks and peripherals
    GPIO_initPins();
    UCS_initModule();
    I2C_initModule();

    CommandSubcommands(BQ769x2_RESET);  // Resets the BQ769x2 registers
    delayUS(60000);
    BQ769x2_Init();  // Configure all of the BQ769x2 register settings
    delayUS(10000);
    CommandSubcommands(FET_ENABLE); // Enable the CHG and DSG FETs
    delayUS(10000);
    CommandSubcommands(SLEEP_DISABLE); // Sleep mode is enabled by default. For this example, Sleep is disabled to
                                       // demonstrate full-speed measurements in Normal mode.

    delayUS(60000); delayUS(60000); delayUS(60000); delayUS(60000);  //wait to start measurements after FETs close


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //Reads Cell, Stack, Pack, LD Voltages, Pack Current and TS1/TS3 Temperatures in a loop
    //This basic example polls the Alarm Status register to see if protections have triggered or new measurements are ready
    //The ALERT pin can also be used as an interrupt to the microcontroller for fastest response time instead of polling
    //In this example the LED on the microcontroller board will be turned on to indicate a protection has triggered and will
    //be turned off if the protection condition has cleared.

        AlarmBits = BQ769x2_ReadAlarmStatus();
        if (AlarmBits & 0x80) {  // Check if FULLSCAN is complete. If set, new measurements are available
            BQ769x2_ReadAllVoltages();
            Pack_Current = BQ769x2_ReadCurrent();
            Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
            Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);
            DirectCommands(AlarmStatus, 0x0080, W);  // Clear the FULLSCAN bit
        }

        if (AlarmBits & 0xC000) {  // If Safety Status bits are showing in AlarmStatus register
            BQ769x2_ReadSafetyStatus(); // Read the Safety Status registers to find which protections have triggered
            if (ProtectionsTriggered & 1) {
                P1OUT |= BIT0;             } // Turn on the LED to indicate Protection has triggered
                DirectCommands(AlarmStatus, 0xF800, W); // Clear the Safety Status Alarm bits.
            }
        else
        {
            if (ProtectionsTriggered & 1) {
                BQ769x2_ReadSafetyStatus();
                if (!(ProtectionsTriggered & 1))
                {
                    P1OUT &= ~BIT0;
                }
            } // Turn off the LED if Safety Status has cleared which means the protection condition is no longer present
        }
        delayUS(20000);  // repeat loop every 20 ms
  }
}


void GPIO_initPins()
{
    // Initialize all pins to output low, except P4.5
    P1OUT = 0x00; P2OUT = 0x00;
    P3OUT = 0x00; P4OUT = 0x00;
    P5OUT = 0x00; P6OUT = 0x00;

    P1DIR = 0xFF; P2DIR = 0xFF;
    P3DIR = 0xFF; P4DIR = 0xFF;
    P5DIR = 0xFF; P6DIR = 0xFF;

    // Configure GPIOs
    GPIO_configPins();

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void GPIO_configPins()
{
    /* Inputs */

    // Button S1
    P4DIR &= ~BIT1;                 // Set P4.1 as input
    P4REN |= BIT1;                  // Enable internal resistor
    P4OUT |= BIT1;                  // Set pull-up on this pin

    // Button S2
    P2DIR &= ~BIT3;                 // Set P2.3 as input
    P2REN |= BIT3;                  // Enable internal resistor
    P2OUT |= BIT3;                  // Set pull-up on this pin

    /* Outputs */

    // P1.0 (LED1)
    P1OUT &= ~BIT0;                 // Set P1.0 to low
    P1DIR |= BIT0;                  // Set P1.0 to output direction

    // P6.6 (LED2)
    P6OUT &= ~BIT6;                 // Set P6.6 to low
    P6DIR |= BIT6;                  // Set P6.6 to output direction

}

void UCS_initModule()
{
#ifdef __ENABLE_XT1__
    // Configure XT1 pins
    P2SEL0 |= BIT6 | BIT7;
    do
    {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);     // Clear XT1 and DCO fault flags
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
#endif

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    __bis_SR_register(SCG0);                           // Disable FLL
#ifdef __ENABLE_XT1__
    CSCTL3 |= SELREF__XT1CLK;                          // Set XT1 as FLL reference source
#else
    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
#endif
    CSCTL0 = 0;                                        // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                            // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // Enable FLL
    Software_Trim();                                   // Software Trim to get the best DCOFTRIM value
#ifdef __ENABLE_XT1__
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;          // MCLK = SMCLK = DCO; ACLK = XT1 (external)
#else
    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;         // MCLK = SMCLK = DCO; ACLK = REFO (internal)
#endif
}

void I2C_initModule()
{
    // Configure I2C pins
    P4SEL0 |= BIT6 | BIT7;
    P4SEL1 &= ~(BIT6 | BIT7);

    // Configure I2C peripheral
    UCB1CTLW0 = UCSWRST;                                    // Enable SW reset
    UCB1CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C host mode, SMCLK
    UCB1BRW = 40;                                           // fSCL = SMCLK/40 = ~400kHz
    UCB1I2CSA = DEV_ADDR;                            // Device Address
    UCB1CTLW0 &= ~UCSWRST;                                  // Clear SW reset, resume operation
    UCB1IE |= UCNACKIE;
}

void UART_initPins()
{
    // Configure UART pins, module is configured in HAL_GUIComm_UART_FR235x.c
    P4SEL0 |= BIT2 | BIT3;
    P4SEL1 &= ~(BIT2 | BIT3);
}


void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * CPU_FREQ_MHZ);  // Wait FLL lock status (FLLUNLOCK) to be stable
                                                            // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}

//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B1_VECTOR))) USCI_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  // Must read from UCB1RXBUF
  uint8_t rx_val = 0;
  switch(__even_in_range(UCB1IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB1RXBUF;
        if (RXByteCtr)
        {
          ReceiveBuffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB1CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB1IE &= ~UCRXIE;
          MasterMode = IDLE_MODE;
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
        break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        switch (MasterMode)
        {
          case TX_REG_ADDRESS_MODE:
              UCB1TXBUF = TransmitRegAddr;
              if (RXByteCtr)
                  MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  MasterMode = TX_DATA_MODE;        // Continue to transmission with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:
              UCB1IE |= UCRXIE;                     // Enable RX interrupt
              UCB1IE &= ~UCTXIE;                    // Disable TX interrupt
              UCB1CTLW0 &= ~UCTR;                   // Switch to receiver
              MasterMode = RX_DATA_MODE;            // State state is to receive data
              UCB1CTLW0 |= UCTXSTT;                 // Send repeated start
              if (RXByteCtr == 1)
              {
                  // Must send stop since this is the N-1 byte
                  while((UCB1CTLW0 & UCTXSTT));
                  UCB1CTLW0 |= UCTXSTP;             // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr)
              {
                  UCB1TXBUF = TransmitBuffer[TransmitIndex++];
                  TXByteCtr--;
              }
              else
              {
                  // Done with transmission
                  UCB1CTLW0 |= UCTXSTP;                 // Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB1IE &= ~UCTXIE;                    // Disable TX interrupt
                  __bic_SR_register_on_exit(CPUOFF);    // Exit LPM0
              }
              break;

          default:
              __no_operation();
              break;
        }
        break;
    default: break;
  }
}
