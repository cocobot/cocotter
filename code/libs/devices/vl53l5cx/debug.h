#ifndef VL53L5CX_API_H
#define VL53L5CX_API_H


#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define VL53L5CX_NB_TARGET_PER_ZONE 1

#define VL53L5CX_NB_TARGET_PER_ZONsE2 1

typedef struct VL53L5CX_Platform {
  uint16_t address;
  void *self_ptr;
  bool (*write_i2c)(void *self_ptr, uint16_t reg_addr, const uint8_t *data, uintptr_t size);
  bool (*read_i2c)(void *self_ptr, uint16_t reg_addr, uint8_t *data, uintptr_t size);
} VL53L5CX_Platform;

uint8_t VL53L5CX_WrByte(struct VL53L5CX_Platform *platform, uint16_t register_addr, uint8_t value);

uint8_t VL53L5CX_WrMulti(struct VL53L5CX_Platform *platform,
                         uint16_t register_addr,
                         const uint8_t *p_values,
                         uint32_t size);

uint8_t VL53L5CX_RdByte(struct VL53L5CX_Platform *platform,
                        uint16_t register_addr,
                        uint8_t *p_value);

uint8_t VL53L5CX_RdMulti(struct VL53L5CX_Platform *platform,
                         uint16_t register_addr,
                         uint8_t *p_values,
                         uint32_t size);

uint8_t VL53L5CX_WaitMs(struct VL53L5CX_Platform *_platform, uint32_t time_ms);

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);

#endif
