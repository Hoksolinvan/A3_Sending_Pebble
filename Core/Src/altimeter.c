#include "altimeter.h"
#include "stm32wlxx_hal_i2c.h"
#include "sys_app.h"
#include "utilities_conf.h"
#include <stdint.h>




extern I2C_HandleTypeDef hi2c2;


static struct alt_calib_data calib_data;




alt_status_t alt_read_reg(alt_reg reg_addrss, uint8_t *data, uint8_t len){

    if( HAL_I2C_Mem_Read(&hi2c2, ALT_ADDRSS, (uint8_t)reg_addrss, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT) == HAL_OK){
        return ALT_I2C_OK;
    }
    else{
        return ALT_I2C_ERROR;
    }

}

alt_status_t alt_write_reg(alt_reg reg_addrss, uint8_t *data, uint8_t len){

    if( HAL_I2C_Mem_Write(&hi2c2, ALT_ADDRSS, (uint8_t)reg_addrss, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT) == HAL_OK){
        return ALT_I2C_OK;
    }
    else{
        return ALT_I2C_ERROR;
    }

}

// Integer/fixed-point temperature compensation (BMP3 datasheet, Bosch integer path).
// Uses the RAW calibration coefficients directly; the datasheet scaling factors
// Returns temperature in hundredths of a degree Celsius (e.g. 2534 => 25.34 C).
static int64_t alt_temp_comp(uint32_t uncomp_temp, struct alt_calib_data *calib_data){
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;

    partial_data1 = (int64_t)uncomp_temp - (256 * (int64_t)calib_data->par_t1);
    partial_data2 = (int64_t)calib_data->par_t2 * partial_data1;
    partial_data3 = partial_data1 * partial_data1;
    partial_data4 = partial_data3 * (int64_t)calib_data->par_t3;
    partial_data5 = (partial_data2 * 262144) + partial_data4;
    partial_data6 = partial_data5 / 4294967296LL;   // /2^32 
    //Store linearized temperature; pressure compensation needs it 
    calib_data->t_lin = partial_data6;
    return (partial_data6 * 25) / 16384;             /* hundredths of degC */
}

// Integer/fixed-point pressure compensation (BMP3 datasheet, Bosch integer path).
// Returns pressure in Pa as Q24.8 fixed point (divide by 256 for whole Pa).
static uint64_t alt_compensate_pressure(uint32_t uncomp_press, struct alt_calib_data *calib_data)
{
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;

    partial_data1 = calib_data->t_lin * calib_data->t_lin;
    partial_data2 = partial_data1 / 64;
    partial_data3 = (partial_data2 * calib_data->t_lin) / 256;
    partial_data4 = ((int64_t)calib_data->par_p8 * partial_data3) / 32;
    partial_data5 = ((int64_t)calib_data->par_p7 * partial_data1) * 16;
    partial_data6 = ((int64_t)calib_data->par_p6 * calib_data->t_lin) * 4194304;
    offset = ((int64_t)calib_data->par_p5 * 140737488355328LL) + partial_data4 + partial_data5 + partial_data6;

    partial_data2 = ((int64_t)calib_data->par_p4 * partial_data3) / 32;
    partial_data4 = ((int64_t)calib_data->par_p3 * partial_data1) * 4;
    partial_data5 = ((int64_t)calib_data->par_p2 - 16384) * calib_data->t_lin * 2097152;
    sensitivity = (((int64_t)calib_data->par_p1 - 16384) * 70368744177664LL) + partial_data2 + partial_data4 + partial_data5;

    partial_data1 = (sensitivity / 16777216) * (int64_t)uncomp_press;
    partial_data2 = (int64_t)calib_data->par_p10 * calib_data->t_lin;
    partial_data3 = partial_data2 + (65536 * (int64_t)calib_data->par_p9);
    partial_data4 = (partial_data3 * (int64_t)uncomp_press) / 8192;

    //divide by 10 then multiply by 10 to avoid intermediate 64-bit overflow 
    partial_data5 = ((int64_t)uncomp_press * (partial_data4 / 10)) / 512;
    partial_data5 = partial_data5 * 10;
    partial_data6 = (int64_t)((uint64_t)uncomp_press * (uint64_t)uncomp_press);
    partial_data2 = ((int64_t)calib_data->par_p11 * partial_data6) / 65536;
    partial_data3 = (partial_data2 * (int64_t)uncomp_press) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    comp_press = ((uint64_t)partial_data4 * 25) / 1099511627776ULL;

    return comp_press;  
}


alt_status_t alt_config(alt_sensor_t * altimeter){
    // Will use the drone sugggested settings fornow (found in datasheet), can change later if needed
    // normal mode, standard resolution, 0x8 pressure, 0x1 temp, IIR filter 2, 570 microA, 50 Hz 11 RMS noise pg 17
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Configuring Altimeter..\r\n");
    uint8_t ret;

   //check communication with altimeter by reading the event register and checking for POR_DETECTED bit
    uint8_t alt_chip_id;
    ret = alt_read_reg(ALT_CHIP_ID, &alt_chip_id, 1);
    if(ret != ALT_I2C_OK) return ret;
    if(alt_chip_id != CHIP_ID){
        APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Trouble communicating with BMP90...\r\n");
        altimeter->i2c_status = ALT_I2C_ERROR;
        return ALT_I2C_ERROR;
    }
 
    //read calibration data from 0x30 to 0x57
    uint8_t calib_buf[21];
    ret = alt_read_reg(0x31, calib_buf, sizeof(calib_buf));
    if (ret != ALT_I2C_OK) return ret;

    calib_data.par_t1  = (uint16_t)(calib_buf[1]  << 8 | calib_buf[0]);
    calib_data.par_t2  = (uint16_t)(calib_buf[3]  << 8 | calib_buf[2]);
    calib_data.par_t3  = (int8_t)   calib_buf[4];
    calib_data.par_p1  = (int16_t) (calib_buf[6]  << 8 | calib_buf[5]);
    calib_data.par_p2  = (int16_t) (calib_buf[8]  << 8 | calib_buf[7]);
    calib_data.par_p3  = (int8_t)   calib_buf[9];
    calib_data.par_p4  = (int8_t)   calib_buf[10];
    calib_data.par_p5  = (uint16_t)(calib_buf[12] << 8 | calib_buf[11]);
    calib_data.par_p6  = (uint16_t)(calib_buf[14] << 8 | calib_buf[13]);
    calib_data.par_p7  = (int8_t)   calib_buf[15];
    calib_data.par_p8  = (int8_t)   calib_buf[16];
    calib_data.par_p9  = (int16_t) (calib_buf[18] << 8 | calib_buf[17]);
    calib_data.par_p10 = (int8_t)   calib_buf[19];
    calib_data.par_p11 = (int8_t)   calib_buf[20];

    //print coeffificents, pressure seems to bee too low. 39KPa, should be 100KPa..
    // temp seems reasonable at 28sih C
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "CALIB p1=%d p2=%d p3=%d p4=%d p5=%u p6=%u\r\n",
            (int)calib_data.par_p1, (int)calib_data.par_p2, (int)calib_data.par_p3,
            (int)calib_data.par_p4, (unsigned)calib_data.par_p5, (unsigned)calib_data.par_p6);
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "CALIB p7=%d p8=%d p9=%d p10=%d p11=%d\r\n",
            (int)calib_data.par_p7, (int)calib_data.par_p8, (int)calib_data.par_p9,
            (int)calib_data.par_p10, (int)calib_data.par_p11);


    // configure mode, OSR, ODR
    uint8_t osr = (ALT_OVERSAMPLING_X1_TEMP << 3) | ALT_PRESSURE_OVERSAMPLING_X8;
    uint8_t odr = ALT_ODR_50HZ;
    uint8_t pwr = (ALT_NORMAL_MODE << 4) | 0x03; // normal mode + enable temp & pressure

    ret = alt_write_reg(ALT_OSR, &osr, 1);
    if(ret != ALT_I2C_OK) return ret;
    ret = alt_write_reg(ALT_ODR, &odr, 1);
    if(ret != ALT_I2C_OK) return ret;
    ret = alt_write_reg(ALT_PWR_CTRL, &pwr, 1);
    if(ret != ALT_I2C_OK) return ret;
    altimeter->mode = ALT_NORMAL_MODE;

    
    // uint8_t cfg_rb[3] = {0};
    // alt_read_reg(ALT_PWR_CTRL, cfg_rb, 3);
    // uint8_t err_rb = 0;
    // alt_read_reg(ALT_ERR_REG, &err_rb, 1);
    // APP_LOG(TS_OFF, VLEVEL_ALWAYS, "PWR_CTRL=%02X OSR=%02X ODR=%02X ERR=%02X\r\n", cfg_rb[0], cfg_rb[1], cfg_rb[2], err_rb);



    altimeter->i2c_status = ret;
    return ALT_I2C_OK;
}

alt_status_t alt_get_data(alt_sensor_t * altimeter){

    //check flag if data is ready
    uint8_t status;
    uint8_t ret = alt_get_status(&status);
    if (ret != ALT_I2C_OK) return ret;
    altimeter->status = status;
    
    

    if((status & ALT_DRDY_PRESS_MSK) && (status & ALT_DRDY_TEMP_MSK)){
        APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Data ready for reading\r\n");
    }
    else{
        APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Data not ready yet\r\n");
        return ALT_NOT_READY;
    }

    //read presseure and temp data  
    uint8_t pressure_data[3];
    uint8_t temp_data[3];
    ret = alt_read_reg(ALT_PRS_DATA_0, pressure_data, 3);
    if(ret != ALT_I2C_OK) return ret;
    ret = alt_read_reg(ALT_TEMP_DATA_3, temp_data,   3);
    if(ret != ALT_I2C_OK) return ret;


    // Data registers are little-endian
    uint32_t pressure_raw = ((uint32_t)pressure_data[2] << 16) | ((uint32_t)pressure_data[1] << 8) | pressure_data[0];
    uint32_t temp_raw     = ((uint32_t)temp_data[2] << 16)     | ((uint32_t)temp_data[1] << 8)     | temp_data[0];

    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Uncompensated pressure: %lu\r\n", pressure_raw);
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Uncompensated temperature: %lu\r\n", temp_raw);

    //compensate the data 
    int64_t  temp_comp   = alt_temp_comp(temp_raw, &calib_data);               // hundredths of degC
    uint64_t press_q24_8 = alt_compensate_pressure(pressure_raw, &calib_data); // Q24.8 Pa
    uint32_t press_pa    = (uint32_t)(press_q24_8 / 256);                      // whole Pa

    int32_t temp_whole = (int32_t)(temp_comp / 100);
    int32_t temp_frac  = (int32_t)(temp_comp % 100);
    if (temp_frac < 0) temp_frac = -temp_frac;

    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Compensated pressure: %lu Pa\r\n", press_pa);
    APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Compensated temperature: %ld.%02ld C\r\n",
            (long)temp_whole, (long)temp_frac);

    return ALT_I2C_OK;
}

alt_status_t alt_get_status(uint8_t * status_check){

    alt_reg status_reg = ALT_STATUS;
    uint8_t ret = alt_read_reg(status_reg, status_check, 1);
    if(ret != ALT_I2C_OK) APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Error reading Alt. status reg.");
    return ret;
}

alt_status_t alt_get_event(uint8_t * event_check){

    alt_reg event_reg = ALT_EVENT;
    uint8_t ret = alt_read_reg(event_reg, event_check, 1);
    if(ret == ALT_I2C_OK) APP_LOG(TS_OFF, VLEVEL_ALWAYS, "ALT_EVENT: %02X\r\n", *event_check);
    return ret;
}

alt_status_t alt_get_error(uint8_t * device_errors){

    alt_status_t error_status = 0;
    alt_reg error_reg = ALT_ERR_REG;
    uint8_t ret = alt_read_reg(error_reg, &error_status, 1);
    if(ret != ALT_I2C_OK){
        APP_LOG(TS_OFF, VLEVEL_ALWAYS, "Error reading Alt. error reg.");
        return ret;
    }
    else{
        *device_errors &= CLEAR_ALT_ERRORS;
        *device_errors |= error_status;
    }
    return ret;
    
}

alt_status_t alt_set_sleep_mode(){
    return ALT_I2C_OK;
}

alt_status_t alt_get_calib_data(){
    return ALT_I2C_OK;
}




void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
        if (hi2c->Instance == I2C2) {
        // the read is done, rx_buf is filled — signal your task
    }
}