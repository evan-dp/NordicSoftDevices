/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/

#ifndef DSI_UTILITY_H
#define DSI_UTILITY_H

#include <stdbool.h>
#include <stdint.h>
#include "appconfig.h"

/**
 * @brief Read unsigned 16-bit from buffer (little endian)
 */
uint16_t DSI_GetUShort(uint8_t *pucData);

/**
 * @brief Read unsigned 32-bit from buffer (little endian)
 */
uint32_t DSI_GetULong(uint8_t *pucData);

/**
 * @brief Buffer copy utility
 */
void DSI_memcpy(uint8_t *pucDest, uint8_t *pucSrc, uint8_t ucSize);

/**
 * @brief Buffer set utility
 */
void DSI_memset(uint8_t *pucDest, uint8_t ucValue, uint8_t ucSize);

/**
 * @brief Buffer compare utility
 */
bool DSI_memcmp(uint8_t *pucSrc1, uint8_t *pucSrc2, uint8_t ucSize);

/**
  @brief Read address to force empty the system bus write buffer
*/
static void DSI_SystemBusWriteBufferEmpty(const volatile uint32_t * pulAddress)
{
#ifdef NRF51
  (void)pulAddress;
#endif
#ifdef NRF52
  uint32_t temp = *pulAddress;
  (void)temp;
#endif
}

#endif // DSI_UTILITY_H
