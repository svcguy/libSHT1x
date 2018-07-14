/** @file sht1x.c
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

#include "sht1x.h"
#include "main.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

// *********************************************************
// Private function prototypes
// *********************************************************
int8_t      sht1x_write_byte( uint8_t value );
int8_t      sht1x_read_byte( uint8_t ack );
void        sht1x_transstart( void );
void        sht1x_connectionreset( void );
int8_t      sht1x_read_statusreg( uint8_t *value, uint8_t *checksum );
int8_t      sht1x_write_statusreg( uint8_t *value );
int8_t      sht1x_measure( uint8_t *value, uint8_t *checksum, uint8_t mode );

// *********************************************************
// Some constants for conversion
// *********************************************************
#define C1  (-2.0468f)          // for 12 Bit RH
#define C2  (+0.0367f)          // for 12 Bit RH
#define C3  (-0.0000015955f)    // for 12 Bit RH
#define T1  (+0.01f)            // for 12 Bit RH
#define T2  (+0.00008f)         // for 12 Bit RH
#define D1  (+0.01f)            // for 14 Bit Temp
#define D2  (+39.65f)           // for degC @ 3.3V

// *********************************************************
// Configs for the DAT pin for faster switching
// *********************************************************
GPIO_InitTypeDef shtDatIsOutput;
GPIO_InitTypeDef shtDatIsInput;

int8_t sht1x_write_byte( uint8_t value )
// writes a byte on the Sensibus and checks the acknowledge 
{
    uint8_t i, x;
    uint8_t error = 0;

    // Setup DAT pin for output
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsOutput );

    for( i = 0x80; i > 0; i/=2 )
    {   // Shift Bit for masking
        if( i & value )
        {   // Bit is high
            HAL_GPIO_WritePin( SHT_DAT_GPIO_Port, SHT_DAT_Pin, GPIO_PIN_SET );
        }
        else
        {   // Bit is low
            HAL_GPIO_WritePin( SHT_DAT_GPIO_Port, SHT_DAT_Pin, GPIO_PIN_RESET );
        }
        
        // Observe setup time
        for( x = 0; x < SETUPHOLDCYCLES; x++ )
        {
            asm("NOP");
        }

        // SCK = 1
        HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_SET );
        
        // Pulsewidth ~5us
        for( x = 0; x < PULSEWIDTHCYCLES; x++ )
        {
            asm("NOP");
        }

        // SCK = 0
        HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

        // Observe hold time
        for( x = 0; x < SETUPHOLDCYCLES; x++ )
        {
            asm("NOP");
        }
    }
    // CLK #9 for ack
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_SET );

    // Release data line
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsInput );

    // Observe setup time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // Check ACK
    error = HAL_GPIO_ReadPin( SHT_DAT_GPIO_Port, SHT_DAT_Pin );

    // Observe hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // SCK = 0
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

    // Check ACK
    error = HAL_GPIO_ReadPin( SHT_DAT_GPIO_Port, SHT_DAT_Pin );

    return error;
}

int8_t sht1x_read_byte( uint8_t ack )
// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"
{
    uint8_t i, x;
    uint8_t val = 0;

    // Release Data Line
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsInput );

    for( i = 0x80; i > 0; i /= 2 )
    {   // Shift bit for masking
        // SCK = 1
        HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_SET );

        // Observe setup/hold time
        for( x = 0; x < SETUPHOLDCYCLES; x++ )
        {
            asm("NOP");
        }

        // Read bit
        if( HAL_GPIO_ReadPin( SHT_DAT_GPIO_Port, SHT_DAT_Pin ) )
        {
            val = val | i;
        }

        // Pulsewidth ~5us
        for( x = 0; x < PULSEWIDTHCYCLES; x++ )
        {
            asm("NOP");
        }        

        // SCK = 0
        HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

        // Pulsewidth ~5us
        for( x = 0; x < PULSEWIDTHCYCLES - SETUPHOLDCYCLES; x++ )
        {
            asm("NOP");
        }
    }

    //in case of "ack==1" pull down DATA-Line
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsOutput );
    HAL_GPIO_WritePin( SHT_DAT_GPIO_Port, SHT_DAT_Pin, !ack );
    // Observe setup/hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // CLK #9 for ack, SCK=1
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_SET );

    // Pulsewidth ~5us
    for( x = 0; x < PULSEWIDTHCYCLES; x++ )
    {
        asm("NOP");
    }

    // SCK=0
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

    // Observe hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // Release Data Line
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsInput );
    
    return val;
}

void sht1x_transstart( void )
// generates a transmission start 
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
{
    uint8_t x;

    // Initial state
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsOutput );
    // DAT=1
    HAL_GPIO_WritePin( SHT_DAT_GPIO_Port, SHT_DAT_Pin, GPIO_PIN_SET );
    // SCK=0
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

    // Observe setup/hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // SCK=1
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_SET );

    // Observe setup/hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // DAT=0
    HAL_GPIO_WritePin( SHT_DAT_GPIO_Port, SHT_DAT_Pin, GPIO_PIN_RESET );

    // Observe setup/hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // SCK=0
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

    // Pulsewidth ~5us
    for( x = 0; x < PULSEWIDTHCYCLES; x++ )
    {
        asm("NOP");
    }

    // SCK=1
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_SET );

    // Observe setup/hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // DAT=1
    HAL_GPIO_WritePin( SHT_DAT_GPIO_Port, SHT_DAT_Pin, GPIO_PIN_SET );

    // Observe setup/hold time
    for( x = 0; x < SETUPHOLDCYCLES; x++ )
    {
        asm("NOP");
    }

    // SCK=0
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

}

void sht1x_connectionreset( void )
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
{
    uint8_t i, x;

    // Initial state
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsOutput );
    // DAT=1
    HAL_GPIO_WritePin( SHT_DAT_GPIO_Port, SHT_DAT_Pin, GPIO_PIN_SET );
    // SCK=0
    HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );

    // 9 SCK Cycles
    for( i = 0; i < 9; i++ )
    {
        HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_SET );
        // Pulsewidth ~5us
        for( x = 0; x < PULSEWIDTHCYCLES; x++ )
        {
            asm("NOP");
        }
        HAL_GPIO_WritePin( SHT_CLK_GPIO_Port, SHT_CLK_Pin, GPIO_PIN_RESET );
        // Pulsewidth ~5us
        for( x = 0; x < PULSEWIDTHCYCLES; x++ )
        {
            asm("NOP");
        }
    }

    // Transmission Start
    sht1x_transstart();
}

int8_t sht1x_softreset( void )
// resets the sensor by a softreset
{
    int8_t error = 0;

    // Reset communication
    sht1x_connectionreset();

    // Send RESET command
    error = sht1x_write_byte( RESET );

    //error=1 in case of no response form the sensor
    return error;
}

int8_t sht1x_read_statusreg( uint8_t *value, uint8_t *checksum )
// reads the status register with checksum (8-bit)
{
    uint8_t x;
    uint8_t error = 0;

    // Transmission Start
    sht1x_transstart();

    // Small delay
    for( x = 0; x < DELAYCYCLES; x++ )
    {
        asm("NOP");
    }

    // Send command to sensor
    error = sht1x_write_byte( STATUS_REG_R );

    // Small delay
    for( x = 0; x < DELAYCYCLES; x++ )
    {
        asm("NOP");
    }
    
    // Read status register (8bit)
    *value = sht1x_read_byte( ACK );

    // Small delay
    for( x = 0; x < DELAYCYCLES; x++ )
    {
        asm("NOP");
    }

    // Read checksum (8bit)
    *checksum = sht1x_read_byte( NACK );

    // error=1 in case of no response from sensor
    return error;
}

int8_t sht1x_write_statusreg( uint8_t *value )
// writes the status register with checksum (8-bit)
{
    uint8_t x;
    uint8_t error = 0;

    // Transmission Start
    sht1x_transstart();

    // Small delay
    for( x = 0; x < DELAYCYCLES; x++ )
    {
        asm("NOP");
    }

    // Send command to sensor
    error += sht1x_write_byte( STATUS_REG_W );

    // Small delay
    for( x = 0; x < DELAYCYCLES; x++ )
    {
        asm("NOP");
    }

    // Send value of status register
    error += sht1x_write_byte( *value );

    //error>=1 in case of no response form the sensor
    return error;
}

int8_t sht1x_measure( uint8_t *value, uint8_t *checksum, uint8_t mode )
// makes a measurement (humidity/temperature) with checksum
{
    uint8_t error=0;
    uint8_t x;

    // Transmission Start
    sht1x_transstart();

    // Send command to the sensor
    switch( mode )
    {
        case TEMP:
            error += sht1x_write_byte( MEASURE_TEMP );
            break;
        case HUMI:
            error += sht1x_write_byte( MEASURE_HUMI );
            break;
    }

    // Small delay
    for( x = 0; x < DELAYCYCLES; x++ )
    {
        asm("NOP");
    }

    // Setup DAT as input
    HAL_GPIO_Init( SHT_DAT_GPIO_Port, &shtDatIsInput );

    // Wait until sensor has finished the measurement
    while( HAL_GPIO_ReadPin( SHT_DAT_GPIO_Port, SHT_DAT_Pin ) );

    // Read the first byte (MSB)
    *(value + 1) = sht1x_read_byte( ACK );

    // Read the second byte (LSB)
    *(value) = sht1x_read_byte( ACK );

    // Read the checksum
    *checksum = sht1x_read_byte( NACK );

    return error;
}

int8_t sht1x_init( void )
{
    shtDatIsOutput.Pin = SHT_DAT_Pin;
    shtDatIsOutput.Mode = GPIO_MODE_OUTPUT_PP;
    shtDatIsOutput.Speed = GPIO_SPEED_FREQ_HIGH;

    shtDatIsInput.Pin = SHT_DAT_Pin;    
    shtDatIsInput.Mode = GPIO_MODE_INPUT;   
    shtDatIsInput.Speed = GPIO_SPEED_FREQ_HIGH;

    return( sht1x_softreset() );
}

uint8_t sht1x_getData( float *temp, float *humi )
// calculates temperature [°C] and humidity [%RH] 
// input :  humi [Ticks] (12 bit) 
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [°C]
{
    float rh_lin;       // rh_lin:  Humidity linear
    float rh_true;      // rh_true: Temperature compensated humidity
    float t_C;          // t_C   :  Temperature [°C]
    
    value rawTemp, rawHumi;
    uint8_t cSum = 0x00;
    uint8_t error = 0;

    // Measure temperature
    error += sht1x_measure( rawTemp.c, &cSum, TEMP );
    // Delay
    HAL_Delay(1);
    // Measure humidity
    error += sht1x_measure( rawHumi.c, &cSum, HUMI );
    // Check for error
    if( error != 0 )
    {
        return error;
    }
    // Convert to float
    rawTemp.f = (float)rawTemp.i;
    rawHumi.f = (float)rawHumi.i;

    //calc. temperature [°C] from 14 bit temp. ticks @ 3V3
    t_C = ( rawTemp.f * D1 ) - D2;
    //calc. humidity from ticks to [%RH]
    rh_lin = C3*rawHumi.f*rawHumi.f + C2*rawHumi.f + C1;
    //calc. temperature compensated humidity [%RH]
    rh_true = ( t_C - 25.0f ) * ( T1+T2*rawHumi.f ) + rh_lin;
    //cut if the value is outside of the physical possible range
    if(rh_true>100)
    {
        rh_true=100;
    }
    if(rh_true<0.1)
    {
        rh_true=0.1;
    }

    //return temperature [°C]
    *temp = t_C;
    //return humidity[%RH]
    *humi = rh_true;    

    return error;          
}

float sht1x_getDewpoint( float *temp, float *humidity )
// calculates dew point
// input:   humidity [%RH], temperature [°C]
// output:  dew point [°C]
{
    float dewPoint;
    float k;

    // This formula comes from the Sensirion example code
    // I don't have a citiation or any documentation
    k = (log10(*humidity)-2)/0.4343 + (17.62*(*temp))/(243.12+(*temp));
    dewPoint = 243.12*k/(17.62-k);

    return dewPoint;
}

// End of file sht1x.c