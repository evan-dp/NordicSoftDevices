/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>
#include <stdint.h>
#include "ant_parameters.h"
#include "appconfig.h"

/**
 * @brief Application system level initialization
 */
void System_Init(void);

/**
 * @brief Application system reset message
 */
void System_ResetMesg(ANT_MESSAGE *pstTxMessage);

/**
 * @brief Application system reset handler
 */
uint8_t System_Reset(uint8_t ucResetCmd);

/**
 * @brief Application deep sleep configuration handler
 */
uint8_t System_SetDeepSleep(ANT_MESSAGE *pstRxMessage); //counter 30.517 uS Resolution.

/**
 * @brief Used for asynchronous serial suspend function
 */
void System_SetSuspendSleep(void);

/**
 * @brief Application deep sleep handler
 */
void System_DeepSleep(void);

#if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
/**
 * @brief Constructs serial number message and calls function to retrieve serial number.
 */
void System_GetSerialNumberMesg(ANT_MESSAGE *pstTxMessage);
/**
 * @brief Retrieves lower 32-bits of 64-bit Nordic device ID stored in FICR. Returns ESN.
 */
void System_GetSerialNumber(uint8_t *pucESN);
#endif // !SERIAL_NUMBER_NOT_AVAILABLE

#endif // SYSTEM_H
