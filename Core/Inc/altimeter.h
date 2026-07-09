#ifndef ALTIMETER_H
#define ALTIMITER_H

#include <stdint.h>
#include "main.h"
#include "stm32wlxx_hal_def.h"
#include "stm32wlxx_hal_subghz.h"
#include "sys_app.h"

#define ALT_ADDRSS (0x76 << 1)
#define CHIP_ID 0x60


#define I2C_TIMEOUT 100

#define CLEAR_ALT_ERRORS    0x7
#define ALT_FATAL_ERR_MSK   (1 << 0)  
#define ALT_CMD_ERROR_MSK   (1 << 1)    
#define ALT_CONFIG_ERR_MSK  (1 << 2)

#define ALT_CMD_RDY_MSK     (1 << 4)
#define ALT_DRDY_PRESS_MSK  (1 << 5)
#define ALT_DRDY_TEMP_MSK   (1 << 6)


typedef enum {
    ALT_I2C_OK      = 0,
    ALT_I2C_ERROR   = 1,
    ALT_I2C_BUSY    = 2,
    ALT_TIMEOUT     = 3
} alt_comm_t;

typedef enum{
    ALT_FATAL_ERR   = 4,
    ALT_CMD_ERROR   = 5,
    ALT_CONFIG_ERR  = 6 
}alt_error_t;

typedef enum{
    ALT_CMD_RDY     = 0,
    ALT_DRDY_PRESS  = 1,
    ALT_DRDY_TEMP   = 2,
    ALT_NOT_READY   = 3 
}alt_status_t;


//function prototypes
typedef struct alt_calib_data{
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;

    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;

    int64_t t_lin; //linearized temperature (fixed point, shared with pressure comp)
}alt_calib_data;

typedef enum{

    ALT_SLEEP_MODE  = 0,
    ALT_NORMAL_MODE = 1,
    ALT_FORCED_MODE = 2,

}alt_mode_t;

typedef enum{
    ALT_POR_DETECTED = 1,
    ALT_ITF_ACT_PTR  = 2
}alt_event_t;

typedef struct{

    float pressure;
    float temperature;
    uint32_t raw_temp;
    uint32_t raw_press;
    alt_calib_data calib_data;
    alt_comm_t     i2c_status;
    alt_event_t    event;
    alt_error_t    errors;
    alt_status_t   status;
    alt_mode_t     mode;

}alt_sensor_t;


typedef enum {

    ALT_CHIP_ID      = 0x00,   
    ALT_REV_ID       = 0x01,
    ALT_ERR_REG      = 0X02,
    ALT_STATUS       = 0X03,

    ALT_PRS_DATA_0   = 0x04,
    ALT_PRS_DATA_1   = 0x05,   
    ALT_PRS_DATA_2   = 0x06,

    ALT_TEMP_DATA_3  = 0x07,
    ALT_TEMP_DATA_4  = 0x08,
    ALT_TEMP_DATA_5  = 0x09,

    ALT_SENSOR_TIME_0   = 0x0C,
    ALT_SENSOR_TIME_1   = 0x0D,
    ALT_SENSOR_TIME_2   = 0x0E,

    ALT_EVENT           = 0x10,
    ALT_INT_STATUS      = 0x11,
    ALT_FIFO_LENGTH_0   = 0x12,
    ALT_FIFO_LENGTH_1   = 0x13,
    ALT_FIFO_DATA       = 0x14,
    ALT_FIFO_WTM_0      = 0x15,
    ALT_FIFO_WTM_1      = 0x16,
    ALT_FIFO_CONFIG_1   = 0x17,
    ALT_FIFO_CONFIG_2   = 0x18,
    ALT_INT_CTRL        = 0x19,

    ALT_IF_CONF         = 0x1A,
    ALT_PWR_CTRL        = 0x1B,
    ALT_OSR             = 0x1C,
    ALT_ODR             = 0x1D,
    ALT_CONFIG          = 0x1F,

    //calibration data 0x30...0x57

    ALT_CMD             = 0x7E

}alt_reg;

// Power mode
#define ALT_SLEEP_MODE          0x00
#define ALT_FORCED_MODE         0x01
#define ALT_NORMAL_MODE         0x03

// Interface
#define ALT_I2C                 0x00
#define ALT_SPI                 0x01

// Data type
#define ALT_PRESSURE            0x00
#define ALT_TEMPERATURE         0x01

// Pressure oversampling
// Pressure oversampling
#define ALT_PRESSURE_OVERSAMPLING_X1     0x00
#define ALT_PRESSURE_OVERSAMPLING_X2     0x01
#define ALT_PRESSURE_OVERSAMPLING_X4     0x02
#define ALT_PRESSURE_OVERSAMPLING_X8     0x03
#define ALT_PRESSURE_OVERSAMPLING_X16    0x04
#define ALT_PRESSURE_OVERSAMPLING_X32    0x05

// Temperature oversampling
#define ALT_OVERSAMPLING_X1_TEMP    0x00
#define ALT_OVERSAMPLING_X2_TEMP    0x01
#define ALT_OVERSAMPLING_X4_TEMP    0x02
#define ALT_OVERSAMPLING_X8_TEMP    0x03
#define ALT_OVERSAMPLING_X16_TEMP   0x04

#define ALT_ODR_200HZ                 0x00
#define ALT_ODR_100HZ                 0x01
#define ALT_ODR_50HZ                  0x02


alt_status_t alt_read_reg(alt_reg reg_addrss, uint8_t *data, uint8_t len);
alt_status_t alt_write_reg(alt_reg reg_addrss, uint8_t *data, uint8_t len);
alt_status_t alt_config();
alt_status_t alt_get_data();
alt_status_t alt_get_error();
alt_status_t alt_get_status();
alt_status_t alt_set_sleep_mode();
alt_status_t alt_get_calib_data();

#endif