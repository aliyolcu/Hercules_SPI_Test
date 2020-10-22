#include "sys_common.h"
#include "system.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "os_task.h"
#include "os_timer.h"
#include "os_queue.h"
#include "os_semphr.h"
#include "os_list.h"

#include "spi.h"
#include "gio.h"

#define HIGH 1
#define LOW 0

#define BTT2_IN1_PORT   gioPORTA
#define BTT2_IN1_PIN    2U

#define BTT1_IN1_PORT   gioPORTA
#define BTT1_IN1_PIN    3U

#define SPI_CS_PORT     spiPORT3
#define SPI_CS_PIN      0U

#define NUM_SSI_DATA    3

//defýntýon of our mcp
#define    IODIRA    (0x00)      // MCP23x17 I/O Direction Register
#define    IODIRB    (0x01)      // 1 = Input (default), 0 = Output

#define    IPOLA     (0x02)      // MCP23x17 Input Polarity Register
#define    IPOLB     (0x03)      // 0 = Normal (default)(low reads as 0), 1 = Inverted (low reads as 1)

#define    GPINTENA  (0x04)      // MCP23x17 Interrupt on Change Pin Assignements
#define    GPINTENB  (0x05)      // 0 = No Interrupt on Change (default), 1 = Interrupt on Change

#define    DEFVALA   (0x06)      // MCP23x17 Default Compare Register for Interrupt on Change
#define    DEFVALB   (0x07)      // Opposite of what is here will trigger an interrupt (default = 0)

#define    INTCONA   (0x08)      // MCP23x17 Interrupt on Change Control Register
#define    INTCONB   (0x09)      // 1 = pin is compared to DEFVAL, 0 = pin is compared to previous state (default)

#define    IOCON     (0x0A)      // MCP23x17 Configuration Register
//                   (0x0B)      //     Also Configuration Register

#define    GPPUA     (0x0C)      // MCP23x17 Weak Pull-Up Resistor Register
#define    GPPUB     (0x0D)      // INPUT ONLY: 0 = No Internal 100k Pull-Up (default) 1 = Internal 100k Pull-Up

#define    INTFA     (0x0E)      // MCP23x17 Interrupt Flag Register
#define    INTFB     (0x0F)      // READ ONLY: 1 = This Pin Triggered the Interrupt

#define    INTCAPA   (0x10)      // MCP23x17 Interrupt Captured Value for Port Register
#define    INTCAPB   (0x11)      // READ ONLY: State of the Pin at the Time the Interrupt Occurred

#define    GPIOA     (0x12)      // MCP23x17 GPIO Port Register 0x12
#define    GPIOB     (0x13)      // Value on the Port - Writing Sets Bits in the Output Latch

#define    OLATA     (0x14)      // MCP23x17 Output Latch Register
#define    OLATB     (0x15)      // 1 = Latch High, 0 = Latch Low (default) Reading Returns Latch State, Not Port Value!


#define    OPCODEW       (0b01000000)  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3
#define    ADDR_ENABLE   (0b00001000)  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing

#define    address        0

#define ADDR_OPCODEW  (OPCODEW | (address << 1))
#define    ADDR_reg       reg
#define    ADDR_word      word


//uint16_t ADDR_OPCODEW = (OPCODEW | (address << 1));

uint32_t pui32DataTx[NUM_SSI_DATA] = {0x40,0x00,0x00};
uint32_t ui32Index;

//mcp varýable
uint32_t _modeCache   = 0xFFFF;                // Default I/O mode is all input, 0xFFFF
uint32_t _outputCache = 0x0000;                // Default output state is all off, 0x0000
uint32_t _pullupCache = 0x0000;                // Default pull-up state is all off, 0x0000
uint32_t _invertCache = 0x0000;                // Default input inversion state is not inverted, 0x0000


/* Define Task Handles */
xTaskHandle xTask1Handle;

uint16_t TX_Data_Slave[3] = { 0x40, 0x0A, 0x08 };
uint16_t RX_Data_Slave[3];

spiDAT1_t dataconfig1_t;

void spiWrite( uint16_t data ){
    spiTransmitData(spiREG3, &dataconfig1_t, 1, &data);
}

void wordWrite(uint8_t reg, unsigned int word) {  // Accept the start register and word

    gioSetBit(BTT1_IN1_PORT, BTT1_IN1_PIN, HIGH);
    gioSetBit(BTT1_IN1_PORT, BTT1_IN1_PIN, LOW);

    spiWrite(ADDR_OPCODEW);

    spiWrite(reg);

    spiWrite((uint8_t) (word));
    spiWrite((uint8_t) (word >> 8));

    gioSetBit(BTT1_IN1_PORT, BTT1_IN1_PIN, HIGH);
}

void byteWrite(uint8_t reg, uint8_t value) {

    gioSetBit(BTT1_IN1_PORT, BTT1_IN1_PIN, HIGH);
    gioSetBit(BTT1_IN1_PORT, BTT1_IN1_PIN, LOW);

    spiWrite(ADDR_OPCODEW);
    spiWrite(reg);
    spiWrite(value);

    gioSetBit(BTT1_IN1_PORT, BTT1_IN1_PIN, HIGH);
}

void pinMode(unsigned int mode) {
  wordWrite(IODIRA, mode);
  _modeCache = mode;
}

void spiOutput(uint8_t pin, uint8_t value) {
  if (pin < 1 | pin > 16) return;
  if (pin < 1 | pin > 16) return;
  if (value) {
    _outputCache |= 1 << (pin - 1);
  } else {
    _outputCache &= ~(1 << (pin - 1));
  }
  wordWrite(GPIOA, _outputCache);
}

void vTaskALL(void *pvParameters)
{
    dataconfig1_t.CS_HOLD = TRUE;
    dataconfig1_t.WDEL    = FALSE;
    dataconfig1_t.DFSEL   = SPI_FMT_0;
    dataconfig1_t.CSNR    = 0xFE;

    _enable_IRQ();

    byteWrite(IOCON, ADDR_ENABLE);
    pinMode(0x0000);

//    TX_Data_Slave[0] = ADDR_OPCODEW;
//    TX_Data_Slave[1] = IOCON;
//    TX_Data_Slave[2] = ADDR_ENABLE;
//    spiTransmitAndReceiveData(spiREG3, &dataconfig1_t,3, TX_Data_Slave, RX_Data_Slave);
//
//    TX_Data_Slave[0] = ADDR_OPCODEW;
//    TX_Data_Slave[1] = IODIRA;
//    TX_Data_Slave[2] = 0x0000;
//    spiTransmitAndReceiveData(spiREG3, &dataconfig1_t,3, TX_Data_Slave, RX_Data_Slave);

    while(1){

//        TX_Data_Slave[0] = ADDR_OPCODEW;
//        TX_Data_Slave[1] = GPIOA;
//        TX_Data_Slave[2] = 0x01;
//        spiTransmitAndReceiveData(spiREG3, &dataconfig1_t,3, TX_Data_Slave, RX_Data_Slave);
//        vTaskDelay(100);
//
//        TX_Data_Slave[0] = ADDR_OPCODEW;
//        TX_Data_Slave[1] = GPIOA;
//        TX_Data_Slave[2] = 0x00;
//        spiTransmitAndReceiveData(spiREG3, &dataconfig1_t,3, TX_Data_Slave, RX_Data_Slave);
//        vTaskDelay(100);

        spiOutput(1,0);
        vTaskDelay(100);

        spiOutput(1,0);
        vTaskDelay(100);
    }
}

int main(void)
{
    spiInit();
    gioInit();


    /* Create Task 1 */
    xTaskCreate(vTaskALL, "TaskALL", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xTask1Handle);

    vTaskStartScheduler();

    while(1);
}
