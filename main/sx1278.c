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


uint16_t sx1278_readRegister(sx1278_t *psx1278, const uint8_t RegisterNumber) 
{
    esp_err_t ret;
    spi_transaction_t t;
    uint8_t OutgoingData[2];
    uint8_t IncomingData[4]={0,0,0,0};    
    OutgoingData[0]=RegisterNumber & 0x7f;      // ensure MSB is zero (its a read operation)
    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length=8;                                 //Command is 8 bits
    t.tx_buffer=OutgoingData;                   //The data is the cmd itself    
    t.rx_buffer=IncomingData;                   //Buffer for the return data
    t.rxlength = 8;                             //8 bit reply expected
    ret=spi_device_transmit(psx1278->spi, &t);  //Transmit!    
    vTaskDelay(10 / portTICK_RATE_MS);
    assert(ret==ESP_OK);               //Should have had no issues.
    return (IncomingData[0]+IncomingData[1]+IncomingData[2]+IncomingData[3]);
}
void sx1278_writeRegister(sx1278_t *psx1278, const uint8_t RegisterNumber, uint8_t Value) 
{
    esp_err_t ret;
    spi_transaction_t t;
    uint8_t OutgoingData[2];    
    OutgoingData[0]=RegisterNumber | 0x80; // ensure MSB is set (its a write operation)
    OutgoingData[1]=Value;
    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length=16;                                //Command is 8 bits
    t.tx_buffer=OutgoingData;                   //The data is the cmd itself        
    ret=spi_device_transmit(psx1278->spi, &t);  //Transmit!    
    vTaskDelay(10 / portTICK_RATE_MS);
    assert(ret==ESP_OK);                        //Should have had no issues.
    
}
//Initialize the transceiver
void sx1278_init(sx1278_t *psx1278) 
{

    //Initialize non-SPI GPIOs    
    gpio_set_direction(psx1278->ResetPin, GPIO_MODE_OUTPUT);    
    // Set up interrupt pin (for RX/TX complete on SX1278 DIO0 
    
    //install gpio isr service (Work in progress)
    gpio_set_direction(psx1278->IRQPin, GPIO_MODE_INPUT);                
    
    //Reset the transceiver
    gpio_set_level(psx1278->ResetPin, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(psx1278->ResetPin, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
    // put in sleep mode
    sx1278_sleep(psx1278);

    // Switch to LoRa mode
    sx1278_writeRegister(psx1278,REG_OP_MODE, MODE_LONG_RANGE_MODE);
    // set frequency
    sx1278_setFrequency(psx1278,psx1278->_frequency);
    psx1278->_packetIndex = 0;
    psx1278->_implicitHeaderMode = 0;
    
    // set base addresses
    sx1278_writeRegister(psx1278,REG_FIFO_TX_BASE_ADDR, 0);
    sx1278_writeRegister(psx1278,REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    sx1278_writeRegister(psx1278,REG_LNA, sx1278_readRegister(psx1278,REG_LNA) | 0x03);

    // set auto AGC
    sx1278_writeRegister(psx1278,REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    sx1278_setTxPower(psx1278,17,1);

    // put in standby mode
    sx1278_idle(psx1278);    
}
void sx1278_displayRegisters(sx1278_t *psx1278)
{
    uint8_t RegNum=0;
    printf("--SX1278 Register contents--\r\n");
    printf(" RegNum      | Value        \r\n");    
    for (RegNum = 0; RegNum <= 0x70; RegNum++)
    {
        printf("  %02X         |",RegNum);
        printf("  %02X         \r\n",sx1278_readRegister(psx1278,RegNum));
    }    
    printf("----------------------------\r\n");
}


void sx1278_end(sx1278_t *psx1278)
{
  // put in sleep mode
  sx1278_sleep(psx1278);  
}

int sx1278_beginPacket(sx1278_t *psx1278,int implicitHeader)
{
  // put in standby mode
  sx1278_idle(psx1278);

  if (implicitHeader) {
    sx1278_implicitHeaderMode(psx1278);
  } else {
    sx1278_explicitHeaderMode(psx1278);
  }

  // reset FIFO address and paload length
  sx1278_writeRegister(psx1278,REG_FIFO_ADDR_PTR, 0);
  sx1278_writeRegister(psx1278,REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int sx1278_endPacket(sx1278_t *psx1278)
{
  // put in TX mode
  sx1278_writeRegister(psx1278,REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  // wait for TX done
  while((sx1278_readRegister(psx1278,REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

  // clear IRQ's
  sx1278_writeRegister(psx1278,REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  return 1;
}

int sx1278_parsePacket(sx1278_t *psx1278,int size)
{
  int packetLength = 0;
  int irqFlags = sx1278_readRegister(psx1278,REG_IRQ_FLAGS);

  if (size > 0) {
    sx1278_implicitHeaderMode(psx1278);

    sx1278_writeRegister(psx1278,REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    sx1278_explicitHeaderMode(psx1278);
  }

  // clear IRQ's
  sx1278_writeRegister(psx1278,REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    psx1278->_packetIndex = 0;

    // read packet length
    if (psx1278->_implicitHeaderMode) {
      packetLength = sx1278_readRegister(psx1278,REG_PAYLOAD_LENGTH);
    } else {
      packetLength = sx1278_readRegister(psx1278,REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    sx1278_writeRegister(psx1278,REG_FIFO_ADDR_PTR, sx1278_readRegister(psx1278,REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    sx1278_idle(psx1278);
  } else if (sx1278_readRegister(psx1278,REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    sx1278_writeRegister(psx1278,REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    sx1278_writeRegister(psx1278,REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int sx1278_packetRssi(sx1278_t *psx1278)
{
  return (sx1278_readRegister(psx1278,REG_PKT_RSSI_VALUE) - (psx1278->_frequency < 868E6 ? 164 : 157));  
}

float sx1278_packetSnr(sx1278_t *psx1278)
{
  return ((int8_t)sx1278_readRegister(psx1278,REG_PKT_SNR_VALUE)) * 0.25;
}

// size_t sx1278_write(sx1278_t *psx1278,uint8_t byte)
// {
//   return sx1278_write(spi,&byte, sizeof(byte));
// }

size_t sx1278_write(sx1278_t *psx1278,const uint8_t *buffer, size_t size)
{
  int currentLength = sx1278_readRegister(psx1278,REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    sx1278_writeRegister(psx1278,REG_FIFO, buffer[i]);
  }

  // update length
  sx1278_writeRegister(psx1278,REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int sx1278_available(sx1278_t *psx1278)
{
  return (sx1278_readRegister(psx1278,REG_RX_NB_BYTES) - psx1278->_packetIndex);
}

int sx1278_read(sx1278_t *psx1278)
{
  if (!sx1278_available(psx1278)) {
    return -1;
  }

  psx1278->_packetIndex++;

  return sx1278_readRegister(psx1278,REG_FIFO);
}

int sx1278_peek(sx1278_t *psx1278)
{
  if (!sx1278_available(psx1278)) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = sx1278_readRegister(psx1278,REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = sx1278_readRegister(psx1278,REG_FIFO);

  // restore FIFO address
  sx1278_writeRegister(psx1278,REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void sx1278_flush(sx1278_t *psx1278)
{
}
/*
void sx1278_onReceive(sx1278_t *psx1278,void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    sx1278_writeRegister(spi,REG_DIO_MAPPING_1, 0x00);

    attachInterrupt(digitalPinToInterrupt(_dio0), sx1278_onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}
*/
void sx1278_receive(sx1278_t *psx1278,int size)
{
  if (size > 0) {
    sx1278_implicitHeaderMode(psx1278);

    sx1278_writeRegister(psx1278,REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    sx1278_explicitHeaderMode(psx1278);
  }

  sx1278_writeRegister(psx1278,REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void sx1278_idle(sx1278_t *psx1278)
{
  sx1278_writeRegister(psx1278,REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sx1278_sleep(sx1278_t *psx1278)
{
  sx1278_writeRegister(psx1278,REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}


void sx1278_setTxPower(sx1278_t *psx1278,int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    sx1278_writeRegister(psx1278,REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level < 2) {
      level = 2;
    } else if (level > 17) {
      level = 17;
    }

    sx1278_writeRegister(psx1278,REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void sx1278_setFrequency(sx1278_t *psx1278,long frequency)
{
  psx1278->_frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  sx1278_writeRegister(psx1278,REG_FRF_MSB, (uint8_t)(frf >> 16));
  sx1278_writeRegister(psx1278,REG_FRF_MID, (uint8_t)(frf >> 8));
  sx1278_writeRegister(psx1278,REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void sx1278_setSpreadingFactor(sx1278_t *psx1278,int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    sx1278_writeRegister(psx1278,REG_DETECTION_OPTIMIZE, 0xc5);
    sx1278_writeRegister(psx1278,REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    sx1278_writeRegister(psx1278,REG_DETECTION_OPTIMIZE, 0xc3);
    sx1278_writeRegister(psx1278,REG_DETECTION_THRESHOLD, 0x0a);
  }

  sx1278_writeRegister(psx1278,REG_MODEM_CONFIG_2, (sx1278_readRegister(psx1278,REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void sx1278_setSignalBandwidth(sx1278_t *psx1278,long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  sx1278_writeRegister(psx1278,REG_MODEM_CONFIG_1, (sx1278_readRegister(psx1278,REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void sx1278_setCodingRate4(sx1278_t *psx1278,int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  sx1278_writeRegister(psx1278,REG_MODEM_CONFIG_1, (sx1278_readRegister(psx1278,REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void sx1278_setPreambleLength(sx1278_t *psx1278,long length)
{
  sx1278_writeRegister(psx1278,REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  sx1278_writeRegister(psx1278,REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void sx1278_setSyncWord(sx1278_t *psx1278,int sw)
{
  sx1278_writeRegister(psx1278,REG_SYNC_WORD, sw);
}

void sx1278_enableCrc(sx1278_t *psx1278)
{
  sx1278_writeRegister(psx1278,REG_MODEM_CONFIG_2, sx1278_readRegister(psx1278,REG_MODEM_CONFIG_2) | 0x04);
}

void sx1278_disableCrc(sx1278_t *psx1278)
{
  sx1278_writeRegister(psx1278,REG_MODEM_CONFIG_2, sx1278_readRegister(psx1278,REG_MODEM_CONFIG_2) & 0xfb);
}

uint8_t sx1278_random(sx1278_t *psx1278)
{
  return sx1278_readRegister(psx1278,REG_RSSI_WIDEBAND);
}

void sx1278_explicitHeaderMode(sx1278_t *psx1278)
{
  psx1278->_implicitHeaderMode = 0;

  sx1278_writeRegister(psx1278, REG_MODEM_CONFIG_1, sx1278_readRegister(psx1278,REG_MODEM_CONFIG_1) & 0xfe);
}

void sx1278_implicitHeaderMode(sx1278_t *psx1278)
{
  psx1278->_implicitHeaderMode = 1;

  sx1278_writeRegister(psx1278,REG_MODEM_CONFIG_1, sx1278_readRegister(psx1278,REG_MODEM_CONFIG_1) | 0x01);
}

