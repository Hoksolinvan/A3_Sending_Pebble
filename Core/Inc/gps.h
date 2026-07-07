#ifndef GPS_H
#define GPS_H


#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "sys_app.h"
#include "stm32wlxx_hal.h"
#include "usart_if.h"


#define UBX_LAYER_RAM    0x01
#define UBX_LAYER_BBR    0x02
#define UBX_LAYER_FLASH  0x04

// MAX-M10S config keys (verify against your module's interface description)
#define CFG_UART1_BAUDRATE            0x40520001  // U4
#define CFG_RATE_MEAS                 0x30210001  // U2 (ms between measurements)
#define CFG_MSGOUT_NMEA_GGA_UART1     0x209100bb  // U1
#define CFG_MSGOUT_NMEA_GLL_UART1     0x209100ca  // U1
#define CFG_MSGOUT_NMEA_GSA_UART1     0x209100c0  // U1
#define CFG_MSGOUT_NMEA_GSV_UART1     0x209100c5  // U1
#define CFG_MSGOUT_NMEA_RMC_UART1     0x209100ac  // U1
#define CFG_MSGOUT_NMEA_VTG_UART1     0x209100b1  // U1




typedef struct {
    uint8_t header[7]; //$GNRMC OR $GNGGA

    // shared position fields (RMC, GGA)
    float utc;
    float lat;
    uint8_t lat_ns;
    float lon;
    uint8_t lon_ew;

    // RMC
    uint8_t status;
    float speed;
    float course;
    float date;

    // GGA
    uint8_t fix_quality;
    uint8_t num_sats;
    float altitude;
    float hdop;

} gps_t;


typedef enum {
    GPS_SENTENCE_UNKNOWN = 0,
    GPS_SENTENCE_RMC,
    GPS_SENTENCE_GGA,
} gps_sentence_t;
  
extern UART_HandleTypeDef hlpuart1;
extern uint8_t dma_rx_buf[];    // circular DMA landing buffer (defined in usart_if.c)

void gps_parser(gps_t *gps_data, uint8_t *gps_buffer);
gps_sentence_t gps_get_sentence_type(uint8_t *gps_buffer);
void gps_parser_rmc(gps_t *gps_data, uint8_t *gps_buffer);
void gps_parser_gga(gps_t *gps_data, uint8_t *gps_buffer);
void gps_parse_sentence(gps_t *gps_data, uint8_t *gps_buffer, gps_sentence_t sentence_type);
void gps_set_baud(UART_HandleTypeDef *huart, uint32_t new_baud, uint8_t *gps_rx_byte);
void gps_valset(UART_HandleTypeDef *huart, uint32_t key, uint64_t value, uint8_t layers);
void gps_config();
#endif /* GPS_H */