/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BQ769x2Header.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float packVoltage;      // V 단위
    float current;          // mA 단위
    float temperatures[3];  // TS1, TS3, IntTemp (°C)
    uint8_t protection[5];  // OVP, UVP, OCP, OTP, UTP
    float cellVoltages[10]; // V 단위
    uint16_t cellBalancing;
} BMS_Data_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEV_ADDR  0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 0  // 0 for disabled, 1 for enabled
#define MAX_BUFFER_SIZE 10
#define R 0 // Read; Used in DirectCommands and Subcommands functions
#define W 1 // Write; Used in DirectCommands and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function

// UART 전송 관련
BMS_Data_t g_bmsData;
uint32_t lastTransmissionTime = 0;
const uint32_t TRANSMISSION_INTERVAL = 1000; // 1초마다 전송

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t RX_data [2] = {0x00, 0x00}; // used in several functions to store data read from BQ769x2
uint8_t RX_Power[2] = {0};
uint8_t RX_Reg0[2] = {0};
uint8_t RX_Reg12[2] = {0};
uint8_t RX_DeftOff[2] = {0};
uint8_t RX_VCellMode[2] = {0};
uint8_t RX_ProtectionA[2] = {0};
uint8_t RX_ProtectionB[2] = {0};
uint8_t RX_CellBalencing[2] = {0};
uint8_t RX_CuVThreshold[2] = {0};
uint8_t RX_COVThreshold[2] = {0};
uint8_t device_number[2] = {0};
uint8_t RX_Buf;

//used in Subcommands read function
// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage [16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t RX_32Byte [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t Temperature [3] = {0};
uint16_t Stack_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
uint16_t Pack_Current = 0x00;

uint16_t CB_ActiveCells = 0x00;

uint16_t AlarmBits = 0x00;
uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;   // Permanent Fail Status Register A
uint8_t value_PFStatusB;   // Permanent Fail Status Register B
uint8_t value_PFStatusC;   // Permanent Fail Status Register C
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs

uint8_t   UV_Fault = 0;   // under-voltage fault state
uint8_t   OV_Fault = 0;   // over-voltage fault state
uint8_t   SCD_Fault = 0;  // short-circuit fault state
uint8_t   OCD_Fault = 0;  // over-current fault state

uint8_t   OTINT_Fault = 0;
uint8_t   OTD_Fault = 0;
uint8_t   OTC_Fault = 0;

uint8_t PowerMode = 0;

uint8_t ProtectionsTriggered = 0; // Set to 1 if any protection triggers

uint8_t LD_ON = 0;   // Load Detect status bit
uint8_t DSG = 0;   // discharge FET state
uint8_t CHG = 0;   // charge FET state
uint8_t PCHG = 0;  // pre-charge FET state
uint8_t PDSG = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int; // in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Frac;// in BQ769x2_READPASSQ func
uint32_t AccumulatedCharge_Time;// in BQ769x2_READPASSQ func

uint8_t Enable_Protection_A_Read[2] = {0x61, 0x92};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef UART_Transmit_String(UART_HandleTypeDef *huart, char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{

   if( HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 100) != HAL_OK) {
      return -1;
   }
   HAL_Delay(10);
   return 0;
}

int I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{

   if( HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg_addr, 1, reg_data, count, 100) != HAL_OK) {
         return -1;
      }
      HAL_Delay(10);
   return 0;
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
            HAL_Delay(10);
         TX_Buffer[0] = Checksum(TX_RegData, 3);
         TX_Buffer[1] = 0x05; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            HAL_Delay(10);
         break;
      case 2: //2 byte datalength
         TX_RegData[3] = (reg_data >> 8) & 0xff;
         I2C_WriteReg(0x3E, TX_RegData, 4);
         HAL_Delay(10);
         TX_Buffer[0] = Checksum(TX_RegData, 4);
         TX_Buffer[1] = 0x06; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            HAL_Delay(10);
         break;
      case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
         TX_RegData[3] = (reg_data >> 8) & 0xff;
         TX_RegData[4] = (reg_data >> 16) & 0xff;
         TX_RegData[5] = (reg_data >> 24) & 0xff;
         I2C_WriteReg(0x3E, TX_RegData, 6);
         HAL_Delay(10);
         TX_Buffer[0] = Checksum(TX_RegData, 6);
         TX_Buffer[1] = 0x08; //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
            HAL_Delay(10);
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
   HAL_Delay(10);
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
      HAL_Delay(10);
      I2C_ReadReg(0x40, RX_32Byte, 32); //RX_32Byte is a global variable
   }
   else if (type == W) {
      //FET_Control, REG12_Control
      TX_Reg[2] = data & 0xff;
      I2C_WriteReg(0x3E,TX_Reg,3);
      HAL_Delay(10);
      TX_Buffer[0] = Checksum(TX_Reg, 3);
      TX_Buffer[1] = 0x05; //combined length of registers address and data
      I2C_WriteReg(0x60, TX_Buffer, 2);
      HAL_Delay(10);
   }
   else if (type == W2){ //write data with 2 bytes
      //CB_Active_Cells, CB_SET_LVL
      TX_Reg[2] = data & 0xff;
      TX_Reg[3] = (data >> 8) & 0xff;
      I2C_WriteReg(0x3E,TX_Reg,4);
      HAL_Delay(10);
      TX_Buffer[0] = Checksum(TX_Reg, 4);
      TX_Buffer[1] = 0x06; //combined length of registers address and data
      I2C_WriteReg(0x60, TX_Buffer, 2);
      HAL_Delay(10);
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
      HAL_Delay(10);
   }
   if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
      I2C_WriteReg(command,TX_data,2);
      HAL_Delay(10);
   }
}

void Subcommand_WithData(uint16_t command, uint8_t *rx_buf, uint8_t rw)
{
    uint8_t tx_buf[2] = {command & 0xFF, (command >> 8) & 0xFF};

    if (rw == R) {
        I2C_WriteReg(0x3E, tx_buf, 2);
        HAL_Delay(2);
        I2C_ReadReg(0x40, rx_buf, 2);
        HAL_Delay(2);
    }
    // 필요시 write 동작도 추가 가능 (지금은 read만 사용 중)
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
   BQ769x2_SetRegister(REG12Config, 0xFD, 1);

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
   BQ769x2_SetRegister(VCellMode, 0x037B, 2);

   // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
   // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
   // COV (over-voltage), CUV (under-voltage)
   BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);

   // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
   // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
   // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
   BQ769x2_SetRegister(EnabledProtectionsB, 0xF0, 1);

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

   BQ769x2_SetRegister(OTDThreshold, 0x21, 1);

   BQ769x2_SetRegister(OTCThreshold, 0x3C, 1);

   BQ769x2_SetRegister(OTINTThreshold, 0x37, 1); //0x1E = 30, 0x23 = 35, 0x21 = 33

   // Exit CONFIGUPDATE mode  - Subcommand 0x0092

   BQ769x2_SetRegister(CellBalanceMinCellVRelax, 3600, 2);

   BQ769x2_SetRegister(CellBalanceMinDeltaRelax, 50, 1);

   BQ769x2_SetRegister(CellBalanceStopDeltaRelax, 5, 1);

   BQ769x2_SetRegister(CellBalanceMaxCells, 0x02, 1);

   CommandSubcommands(EXIT_CFGUPDATE);
}

//  ********************************* FET Control Commands  ***************************************

void BQ769x2_BOTHOFF () {
   // Disables all FETs using the DFETOFF (BOTHOFF) pin
   // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);  // DFETOFF pin (BOTHOFF) set high
}

void BQ769x2_RESET_BOTHOFF () {
   // Resets DFETOFF (BOTHOFF) pin
   // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // DFETOFF pin (BOTHOFF) set low
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
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);  // Sets RST_SHUT pin
}

void BQ769x2_ReleaseShutdownPin() {
   // Releases the RST_SHUT pin
   // The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // Resets RST_SHUT pin
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
   OTINT_Fault = ((0x40 & RX_data[0])>>6);
   OTD_Fault = ((0x20 & RX_data[0])>>5);
   OTC_Fault = ((0x10 & RX_data[0])>>4);


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
   if(command >= Cell1Voltage && command <= Cell10Voltage) {//Cells 1 through 10 (0x14 to 0x26)
      return (RX_data[1]*256 + RX_data[0]); //voltage is reported in mV
   }
   else {//stack, Pack, LD
      return 10 * (RX_data[1]*256 + RX_data[0]); //voltage is reported in 0.01V units
   }

}
void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{

  int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
  for (int x = 0; x < 10; x++){//Reads all cell voltages
    CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
    cellvoltageholder = cellvoltageholder + 2;
  }
  Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);

  Pack_Voltage = 0;
  for (int y =0; y <10; y++) {
	  Pack_Voltage += CellVoltage[y];
  }

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

void BQ769x2_CheckPowerMode()
{
    DirectCommands(ControlStatus, 0x00, R);  // RX_data[0]에 값 들어감
    PowerMode = (RX_data[0] >> 3) & 0x03;    // 비트 4:3만 추출
    // 0 = NORMAL, 1 = SLEEP, 2 = DEEPSLEEP, 3 = SHUTDOWN
}

void BQ769x2_CheckCellBalancing()
{
    Subcommands(CB_ACTIVE_CELLS, 0x00, R);  // 0x0074
    CB_ActiveCells = ((uint16_t)RX_32Byte[3] << 24) |
                     ((uint16_t)RX_32Byte[2] << 16) |
                     ((uint16_t)RX_32Byte[1] << 8) |
                     RX_32Byte[0];
}

// 실제 BMS 데이터 읽기 (I2C에서)
void BMS_ReadRealData(BMS_Data_t *bmsData) {
    // 전압 데이터 읽기
    BQ769x2_ReadAllVoltages();

    // Pack 전압: 0.01V 단위를 V 단위로 변환
    bmsData->packVoltage = (((Pack_Voltage * 0.001f) - 25.0) / 17.0)*100 ;

    // 셀 전압: mV를 V로 변환
    for (int i = 0; i < 10; i++) {
        bmsData->cellVoltages[i] = CellVoltage[i] * 0.001f; // mV to V
    }

    // 전류 읽기: mA를 A로 변환
    Pack_Current = BQ769x2_ReadCurrent();
    bmsData->current = (int16_t)Pack_Current; // mA to A (signed)

    // 온도 읽기
    bmsData->temperatures[0] = BQ769x2_ReadTemperature(TS1Temperature);
    bmsData->temperatures[1] = BQ769x2_ReadTemperature(TS3Temperature);
    bmsData->temperatures[2] = BQ769x2_ReadTemperature(0x68); // Internal temp

    // 안전 상태 읽기
    BQ769x2_ReadSafetyStatus();

    // Protection 상태 설정 (UV_Fault, OV_Fault는 반전)
    bmsData->protection[0] = OV_Fault; // OVP (반전)
    bmsData->protection[1] = UV_Fault; // UVP (반전)
    bmsData->protection[2] = OCD_Fault; // OCP
    bmsData->protection[3] = (OTC_Fault || OTD_Fault || OTINT_Fault); // OTP
    bmsData->protection[4] = 0; // UTP (해당 신호 없음)

    // 셀 밸런싱 상태
    BQ769x2_CheckCellBalancing();
    bmsData->cellBalancing = CB_ActiveCells;
}

// JSON 형태로 UART 전송
void BMS_SendJSON(BMS_Data_t *bmsData) {
    char jsonBuffer[1024];

    snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{"
        "\"packVoltage\":%.2f,"
        "\"current\":%.3f,"
        "\"temperatures\":{"
            "\"ts1\":%.1f,"
            "\"ts3\":%.1f,"
            "\"intTemp\":%.1f"
        "},"
        "\"protection\":{"
            "\"ovp\":%s,"
            "\"uvp\":%s,"
            "\"ocp\":%s,"
            "\"otp\":%s,"
            "\"utp\":%s"
        "},"
        "\"cellVoltages\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
        "\"cellBalancing\":%d"
        "}\r\n",

        bmsData->packVoltage,
        bmsData->current,
        bmsData->temperatures[0],
        bmsData->temperatures[1],
        bmsData->temperatures[2],
        bmsData->protection[0] ? "true" : "false",
        bmsData->protection[1] ? "true" : "false",
        bmsData->protection[2] ? "true" : "false",
        bmsData->protection[3] ? "true" : "false",
        bmsData->protection[4] ? "true" : "false",
        bmsData->cellVoltages[0], bmsData->cellVoltages[1],
        bmsData->cellVoltages[2], bmsData->cellVoltages[3],
        bmsData->cellVoltages[4], bmsData->cellVoltages[5],
        bmsData->cellVoltages[6], bmsData->cellVoltages[7],
        bmsData->cellVoltages[8], bmsData->cellVoltages[9],
        bmsData->cellBalancing
    );

    UART_Transmit_String(&huart3, jsonBuffer);
}

   // UART 문자열 전송
   HAL_StatusTypeDef UART_Transmit_String(UART_HandleTypeDef *huart, char *str) {
      return HAL_UART_Transmit(huart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
   }

   // BMS 작업 함수
   void BMS_Task(void) {
      uint32_t currentTime = HAL_GetTick();

      if (currentTime - lastTransmissionTime >= TRANSMISSION_INTERVAL) {
         // 실제 BMS 데이터 읽기
         BMS_ReadRealData(&g_bmsData);

         // UART로 JSON 데이터 전송
         BMS_SendJSON(&g_bmsData);

         lastTransmissionTime = currentTime;
      }
   }



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(500);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // RST_SHUT pin set low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // DFETOFF pin (BOTHOFF) set low
  HAL_Delay(10);

  CommandSubcommands(BQ769x2_RESET);  // Resets the BQ769x2 registers
  HAL_Delay(60);
  BQ769x2_Init();  // Configure all of the BQ769x2 register settings
  HAL_Delay(10);
  CommandSubcommands(FET_ENABLE); // Enable the CHG and DSG FETs
  HAL_Delay(10);
  CommandSubcommands(SLEEP_DISABLE); // Sleep mode is enabled by default. For this example, Sleep is disabled to
                                // demonstrate full-speed measurements in Normal mode.

  HAL_Delay(60); HAL_Delay(60); HAL_Delay(60); HAL_Delay(60);  //wait to start measurements after FETs close



  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     AlarmBits = BQ769x2_ReadAlarmStatus();
                       if (AlarmBits & 0x80) {  // Check if FULLSCAN is complete. If set, new measurements are available
                             BQ769x2_ReadAllVoltages();
                             Pack_Current = BQ769x2_ReadCurrent();
                             Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
                             Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);
                             Temperature[2] = BQ769x2_ReadTemperature(0x68);

                          DirectCommands(AlarmStatus, 0x0080, W);  // Clear the FULLSCAN bit
                       }

                       // BMS 데이터 읽기 및 UART 전송 (1초마다)



                       BQ769x2_ReadSafetyStatus();

                       BQ769x2_ReadFETStatus();

                       BQ769x2_CheckCellBalancing();

                       BQ769x2_CheckPowerMode();

                       HAL_I2C_Mem_Write(&hi2c1, 0x10, 0x3E, 1, Enable_Protection_A_Read, 2, 100);
                       HAL_Delay(10);

                       HAL_I2C_Mem_Read(&hi2c1, 0x10, 0x40, 1, &RX_Buf, 1, 100);
                       HAL_Delay(10);

                       BMS_Task();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
