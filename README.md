# libSHT1x
A library for the Sensirion SHT1x for STM32CubeMX HAL Drivers

This is a work in progress and so far is only the first basic working code

* STM32CubeMX - https://www.st.com/en/development-tools/stm32cubemx.html
* SHT1x - https://www.sensirion.com/en/environmental-sensors/humidity-sensors/digital-humidity-sensors-for-accurate-measurements/

This is based mostly on the Sensirion Example code (https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/13_Sample_Codes_Software/Humidity_Sensors/Sensirion_Humidity_Sensors_SHT1x_Sample_Code_C-file.zip)

So far only tested with my setup:

* Adafruit Weather-proof Temperature/Humidity Sensor (https://www.adafruit.com/product/1298)
* Waveshare Open103z Development Board (https://www.waveshare.com/open103z-standard.htm)


#TODO
* It's running on the STM32F103ZET6 @ 8MHz and relies on a bunch of assembly NOPs to generate the timing - change this to use a hardware timer
* It currently ignores the CRC checksum - get this working with either a software implementation or the hardware CRC generator
* Make it non-blocking
