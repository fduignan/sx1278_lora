/* SPI Master interfacing to the SX1278 LoRa transceiver
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "sx1278.h"


#define PIN_NUM_MISO 23
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_RST  22
#define PIN_NUM_IRQ  GPIO_NUM_2

void app_main()
{
    sx1278_t sx1278;
    sx1278.ResetPin = PIN_NUM_RST;
    sx1278.IRQPin = PIN_NUM_IRQ;
    sx1278._frequency = 433123000;
    
    int packetSize = 0;
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1000000,                //Clock out at 1MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=1,                          //We want to be able to queue 7 transactions at a time
        .flags=SPI_DEVICE_HALFDUPLEX        
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the transceiver to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    sx1278.spi = spi;
    sx1278_init(&sx1278);
    sx1278_setSpreadingFactor(&sx1278,12);
    sx1278_setSignalBandwidth(&sx1278,125000);
    sx1278_setCodingRate4(&sx1278,5);
    
    while(1)
    {   
                        
        
        packetSize = 0; 
        while (!packetSize)
            packetSize = sx1278_parsePacket(&sx1278,0);
        if (packetSize) {
            // received a packet
            printf("Received packet '");

            // read packet
            while (sx1278_available(&sx1278)) {
                printf("%c",(char)sx1278_read(&sx1278));
            }

            // print RSSI of packet
            printf("' with RSSI ");
            printf("%d\r\n",sx1278_packetRssi(&sx1278));
            printf("SNR = %f\r\n",sx1278_packetSnr(&sx1278));
        }                
        
    }    
}
