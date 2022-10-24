#include <stdio.h>
#include "include/pzem-driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "soc/soc_caps.h"
#include "soc/uart_struct.h"

#include <esp_log.h>
#include <esp_err.h>

#define TAG "PZEM-driver"
#define PZEM_UART_NUM UART_NUM_1

#define PZEM_UART_BAUD_RATE 9600
#define PZEM_UART_BUFFER_SIZE 256
#define TIMEOUT 100

/**
 * @brief pzem commant
 *
 */
#define REG_VOLTAGE 0x0000
#define REG_CURRENT_L 0x0001
#define REG_CURRENT_H 0X0002
#define REG_POWER_L 0x0003
#define REG_POWER_H 0x0004
#define REG_ENERGY_L 0x0005
#define REG_ENERGY_H 0x0006
#define REG_FREQUENCY 0x0007
#define REG_PF 0x0008
#define REG_ALARM 0x0009

#define CMD_RHR 0x03
#define CMD_RIR 0X04
#define CMD_WSR 0x06
#define CMD_CAL 0x41
#define CMD_REST 0x42

#define WREG_ALARM_THR 0x0001
#define WREG_ADDR 0x0002

#define UPDATE_TIME 200

#define RESPONSE_SIZE 32
#define READ_TIMEOUT 100

#define INVALID_ADDRESS 0x00

/*************************************struct and variables*******************************************/

typedef struct pzemValues_t
{
    float voltage;
    float current;
    float power;
    float energy;
    float frequency;
    float pf;
    uint16_t alarms;

} pzemValues_t;

pzemValues_t pzemValues;

// Pre computed CRC table
static const uint16_t crcTable[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

/*!
 * PZEM004Tv30::CRC16
 *
 * Calculate the CRC16-Modbus for a buffer
 * Based on https://www.modbustools.com/modbus_crc16.html
 *
 * @param[in] data Memory buffer containing the data to checksum
 * @param[in] len  Length of the respBuffer
 *
 * @return Calculated CRC
 */
uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    uint8_t nTemp;         // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= crcTable[nTemp];
    }
    return crc;
}

void initialize_pzem(uint8_t txPin, uint8_t rxPin)
{
    uart_config_t uart_config;
    uart_config.baud_rate = PZEM_UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;
    uart_config.source_clk = UART_SCLK_APB;

    ESP_ERROR_CHECK(uart_driver_install(PZEM_UART_NUM, 256, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(PZEM_UART_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(PZEM_UART_NUM, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


/*!
 * PZEM004Tv30::checkCRC
 *
 * Performs CRC check of the buffer up to len-2 and compares check sum to last two bytes
 *
 * @param[in] data Memory buffer containing the frame to check
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
 * @return is the buffer check sum valid
*/
bool checkCRC(const uint8_t *buf, uint16_t len){
    if(len <= 2) // Sanity check
        return false;

    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}


/*!
 * PZEM004Tv30::setCRC
 *
 * Set last two bytes of buffer to CRC16 of the buffer up to byte len-2
 * Buffer must be able to hold at least 3 bytes
 *
 * @param[out] data Memory buffer containing the frame to checksum and write CRC to
 * @param[in] len  Length of the respBuffer including 2 bytes for CRC
 *
 */
void setCRC(uint8_t *buf, uint16_t len)
{
    if (len <= 2) // Sanity check
        return;

    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF;        // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}

uint8_t receive(uint8_t *resp, uint8_t length)
{
    uint8_t readLength = uart_read_bytes(PZEM_UART_NUM, resp, length, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "read lenght is %d", readLength);

    // Check CRC with the number of bytes read
    if (!checkCRC(resp, readLength))
    {
        ESP_LOGE(TAG, "checksum failed while reading");
        return 0;
    }
    return readLength;
}

/*!
 * PZEM004Tv30::sendCmd8
 *
 * Prepares the 8 byte command buffer and sends
 *
 * @param[in] cmd - Command to send (position 1)
 * @param[in] rAddr - Register address (postion 2-3)
 * @param[in] val - Register value to write (positon 4-5)
 * @param[in] check - perform a simple read check after write
 *
 * @return success
 */
bool sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr)
{
    uint8_t sendBuffer[8]; // Send buffer
    uint8_t respBuffer[8]; // Response buffer (only used when check is true)

    if ((slave_addr == 0xFFFF) ||
        (slave_addr < 0x01) ||
        (slave_addr > 0xF7))
    {
        ESP_LOGW(TAG, "Address out of range");
    }

    sendBuffer[0] = slave_addr; // Set slave address
    sendBuffer[1] = cmd;        // Set command

    sendBuffer[2] = (rAddr >> 8) & 0xFF; // Set high byte of register address
    sendBuffer[3] = (rAddr)&0xFF;        // Set low byte =//=

    sendBuffer[4] = (val >> 8) & 0xFF; // Set high byte of register value
    sendBuffer[5] = (val)&0xFF;        // Set low byte =//=

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, sendBuffer, ESP_LOG_DEBUG, 8);

    setCRC(sendBuffer, 8); // Set CRC of frame

    uart_write_bytes(PZEM_UART_NUM, sendBuffer, 8); // actually sending to the uart

    if (check)
    {
        if (!receive(respBuffer, 8))
        { // if check enabled, read the response
            return false;
        }

        // Check if response is same as send
        for (uint8_t i = 0; i < 8; i++)
        {
            if (sendBuffer[i] != respBuffer[i])
                ESP_LOGE(TAG, "send bytes are not equal to the read bytes");
            return false;
        }
    }
    return true;
}

bool updateValues(uint8_t addr)
{
    static uint8_t response[25];

    // Read 10 registers starting at 0x00 (no check)
    sendCmd8(CMD_RIR, 0x00, 0x0A, false, addr);

    if (receive(response, 25) != 25)
    { // Something went wrong
        return false;
    }

    pzemValues.voltage = ((uint32_t)response[3] << 8 | // Raw voltage in 0.1V
                          (uint32_t)response[4]) /
                         10.0;

    pzemValues.current = ((uint32_t)response[5] << 8 | // Raw current in 0.001A
                          (uint32_t)response[6] |
                          (uint32_t)response[7] << 24 |
                          (uint32_t)response[8] << 16) /
                         1000.0;

    pzemValues.power = ((uint32_t)response[9] << 8 | // Raw power in 0.1W
                        (uint32_t)response[10] |
                        (uint32_t)response[11] << 24 |
                        (uint32_t)response[12] << 16) /
                       10.0;

    pzemValues.energy = ((uint32_t)response[13] << 8 | // Raw Energy in 1Wh
                         (uint32_t)response[14] |
                         (uint32_t)response[15] << 24 |
                         (uint32_t)response[16] << 16) /
                        1000.0;

    pzemValues.frequency = ((uint32_t)response[17] << 8 | // Raw Frequency in 0.1Hz
                            (uint32_t)response[18]) /
                           10.0;

    pzemValues.pf = ((uint32_t)response[19] << 8 | // Raw pf in 0.01
                     (uint32_t)response[20]) /
                    100.0;

    pzemValues.alarms = ((uint32_t)response[21] << 8 | // Raw alarm value
                         (uint32_t)response[22]);

    return true;
}

float getVoltage()
{
    return pzemValues.voltage;
}

float getCurrent()
{
    return pzemValues.current;
}

float getPower()
{
    return pzemValues.power;
}

float getEnergy()
{
    return pzemValues.energy;
}

float getFrequency()
{
    return pzemValues.frequency;
}

float getPF()
{
    return pzemValues.pf;
}

uint16_t getAlarm(){
    return pzemValues.alarms;
}