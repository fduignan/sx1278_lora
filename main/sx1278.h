#include "driver/spi_master.h"
typedef struct {
    uint32_t IRQPin;
    uint32_t ResetPin;
    uint32_t _implicitHeaderMode;
    uint32_t _frequency;
    uint32_t _packetIndex;
    spi_device_handle_t spi;
    
} sx1278_t;
void sx1278_init(sx1278_t *psx1278);
void sx1278_displayRegisters(sx1278_t *psx1278);
void sx1278_writeRegister(sx1278_t *psx1278, const uint8_t RegisterNumber, uint8_t Value);
uint16_t sx1278_readRegister(sx1278_t *psx1278, const uint8_t RegisterNumber);
int sx1278_beginPacket(sx1278_t *psx1278,int implicitHeader);
int sx1278_endPacket(sx1278_t *psx1278);
int sx1278_parsePacket(sx1278_t *psx1278,int size);
int sx1278_packetRssi(sx1278_t *psx1278);
float sx1278_packetSnr(sx1278_t *psx1278);
size_t sx1278_write(sx1278_t *psx1278,const uint8_t *buffer, size_t size);
int sx1278_available(sx1278_t *psx1278);
int sx1278_read(sx1278_t *psx1278);
int sx1278_peek(sx1278_t *psx1278);
void sx1278_flush(sx1278_t *psx1278);
void sx1278_receive(sx1278_t *psx1278,int size);
void sx1278_idle(sx1278_t *psx1278);
void sx1278_sleep(sx1278_t *psx1278);

void sx1278_setTxPower(sx1278_t *psx1278,int level, int outputPin);
void sx1278_setFrequency(sx1278_t *psx1278,long frequency);
void sx1278_setSpreadingFactor(sx1278_t *psx1278,int sf);
void sx1278_setSignalBandwidth(sx1278_t *psx1278,long sbw);
void sx1278_setCodingRate4(sx1278_t *psx1278,int denominator);
void sx1278_setPreambleLength(sx1278_t *psx1278,long length);
void sx1278_setSyncWord(sx1278_t *psx1278,int sw);
void sx1278_enableCrc(sx1278_t *psx1278);
void sx1278_disableCrc(sx1278_t *psx1278);
uint8_t sx1278_random(sx1278_t *psx1278);
void sx1278_implicitHeaderMode(sx1278_t *psx1278);
void sx1278_explicitHeaderMode(sx1278_t *psx1278);

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

// PA config
#define PA_BOOST                 0x80

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1
