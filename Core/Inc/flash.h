#pragma once

// include files
#include <stdint.h>
#include "main.h"


// Winbond Configuration
#define WIN_CS_PORT GPIOA
#define WIN_CS_PIN GPIO_PIN_4



// Instruction opcode
#define WRITE_ENABLE 0x06
#define VOLATILE_SR_WRITE_ENABLE 0x50
#define WRITE_DISABLE 0x04
#define READ_DATA 0x03
#define FAST_READ 0x0B
#define PAGE_PROGRAM 0x02
#define SECTOR_ERASE 0x20
#define BLOCK_ERASE_32K 0x52
#define BLOCK_ERASE_64K 0xD8
#define CHIP_ERASE 0xC7
#define ERASE_PROGRAM_SUSPEND 0x75




// status
typedef enum{
	SREG1 = 0x05,
	SREG2 = 0x35,
	SREG3 = 0x15

} Status_Register_t;

typedef enum{
	WIN_SPI_OK = 0,
	WIN_SPI_ERROR = 1,
	WIN_SPI_BUSY = 2,
	WIN_SPI_TIMEOUT = 3


} spi_write_t;



typedef enum {
    WIN_SREG1_WRITE = 0x01,
    WIN_SREG2_WRITE = 0x31,
    WIN_SREG3_WRITE = 0x11
} win_sreg_write_t;




// function prototypes

spi_write_t Win_Write_Enable(void);
spi_write_t Win_Write_Enable_Volatile(void);
spi_write_t Win_Write_Disable(void);
spi_write_t Win_Read_Status_Register(uint8_t* sr_value, Status_Register_t sreg);
spi_write_t Win_Write_Status_Register(Status_Register_t sreg, const uint8_t sreg_bytes);
spi_write_t Win_Write_Status_Register_Volatile(Status_Register_t sreg, const uint8_t sreg_bytes);
spi_write_t Win_Read_Data(const uint8_t address[], uint8_t result[], int reads);
spi_write_t Win_Fast_Read_Data(const uint8_t address[], uint8_t result[], int reads);
spi_write_t WIN_Page_Program(const uint8_t address[], uint8_t data[], int n);
spi_write_t Win_Sector_Erase(const uint8_t address[]);
spi_write_t Block_Erase_32K(const uint8_t address[]);
spi_write_t Block_Erase_64K(const uint8_t address[]);
spi_write_t Win_Chip_Erase();
