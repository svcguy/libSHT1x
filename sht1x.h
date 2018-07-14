/** @file sht1x.h
* 
* @brief    A driver for the Sensirion SHT1X sensor
*           https://www.sensirion.com/en/environmental-sensors/humidity-sensors/digital-humidity-sensors-for-accurate-measurements/
*           Based in part on sample code from Sensirion
*           https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/13_Sample_Codes_Software/Humidity_Sensors/Sensirion_Humidity_Sensors_SHT1x_Sample_Code_C-file.zip
*           
*           Sensirion Author: MST
*           Sensirion Copyright: (c) Sensirion AG
*
*           Adapted to be used with the STM32CubeMX libraries for STM32 microcontrollers
*           https://www.st.com/en/development-tools/stm32cubemx.html
*           
*           Author:  Andy Josephson, 2018
* @par      
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __sht1x_H
#define __sht1x_H

#include <stdint.h>

#ifdef USE_HAL_DRIVER
#include "gpio.h"
#else
#error "The VEML6070 driver requires the use of STM32CubeMX HAL drivers"
#endif /* USE_HAL_DRIVER */

#include <math.h>

typedef union
{
    uint16_t    i;
    uint8_t     c[2];
    float       f;
} value;

enum
{
    TEMP = 0,
    HUMI
};

#define NACK    0
#define ACK     1

                            //adr  command  r/w
#define STATUS_REG_W 0x06   //000   0011    0
#define STATUS_REG_R 0x07   //000   0011    1
#define MEASURE_TEMP 0x03   //000   0001    1
#define MEASURE_HUMI 0x05   //000   0010    1
#define RESET        0x1e   //000   1111    0

#define PULSEWIDTHCYCLES    50
#define SETUPHOLDCYCLES     15
#define DELAYCYCLES         100

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @brief Initializes the library
  * @param None
  * @retval 1 on successs, 0 on failure
  */
int8_t sht1x_init( void );

/**
  * @brief Resets the sensor
  * @param None
  * @retval 1 on successs, 0 on failure
  */
int8_t sht1x_softreset( void );

/**
  * @brief Converts raw temperature and humidity to degC and %rh
  * @param *temp - pointer to a float to store read temperature
  * @param *humi - pointer to a float to store compensted humidity
  * @retval 0 on success, >=1 on failure
  */
uint8_t sht1x_getData( float *temp, float *humi );

/**
  * @brief Calculates the dewpoint
  * @param *temp - Pointer to temperature in degC
  * @param *humidity - Pointer to humidity in %rh
  * @retval 
  */
float sht1x_getDewpoint( float *temp, float *humidity );

#ifdef __cplusplus
}
#endif
#endif /*__sht1x_H */

// End of file sht1x.h