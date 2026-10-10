#include "crc.h"

/**
 * CRC16_Modbus
 * Modbus RTU 标准 CRC16 校验
 * 多项式: 0xA001（0x8005 反转）, 初始值: 0xFFFF
 * 结果字节序: 低字节在前，高字节在后（小端）
 */
uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
