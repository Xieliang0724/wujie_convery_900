#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Modbus RTU CRC16 (poly 0xA001, init 0xFFFF) */
uint16_t CRC16_Modbus(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRC_H__ */
