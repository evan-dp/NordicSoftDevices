 /*
 * Dynastream Innovations Inc.
 * Cochrane, AB, CANADA
 *
 * Copyright (c) 2013 Dynastream Innovations Inc.
 * All rights reserved. This software may not be reproduced by
 * any means without express written approval of Dynastream
 * Innovations Inc.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "appconfig.h"
#include "boardconfig.h"
#include "command.h"
#include "dsi_utility.h"
#include "global.h"
#include "main.h"
#include "serial.h"
#include "svc_app.h"
#include "system.h"

#include "nrf.h"
#include "nrf_delay.h"

#include "ant_error.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "nrf_error.h"
#include "nrf_error_sdm.h"
#include "nrf_error_soc.h"
#include "nrf_mbr.h"
#if defined (NRF52)
#include "nrf_nvic.h"
#endif
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_svc.h"


/**
 * @brief Application system level initialization
 */
void System_Init(void)
{
   //Default pin settings.
   NRF_GPIO->OUT = PIN_OUT_INIT;
   NRF_GPIO->DIR = PIN_DIR_INIT;

   SCB->SCR |= SCB_SCR_SEVONPEND_Msk; // allow wakeup from enabled events and all interrupts (including disabled)
}

/**
 * @brief Application system reset message
 */
void System_ResetMesg(ANT_MESSAGE *pstTxMessage)
{
   uint32_t ulErrorCode;
   uint32_t ulReason;
   uint8_t ucResetMesg = 0;  // Default power on reset

   ulErrorCode = sd_power_reset_reason_get((uint32_t*)&ulReason);
   APP_ERROR_CHECK(ulErrorCode);

//   if ( ulReason & POWER_RESETREAS_OFF_Msk) // from Power OFF mode (lowest powerdown). TODO: not sure is this is correct
//      ucResetMesg |= RESET_SUSPEND;
   if ( ulReason & POWER_RESETREAS_LOCKUP_Msk) // it looks like nrf_nvic_SystemReset generates this instead of SREQ
      ucResetMesg |= RESET_CMD;
   if ( ulReason & POWER_RESETREAS_SREQ_Msk) // from System reset req
      ucResetMesg |= RESET_CMD;
   if ( ulReason & POWER_RESETREAS_DOG_Msk) // from watchdog
      ucResetMesg |= RESET_WDT;
   if ( ulReason & POWER_RESETREAS_RESETPIN_Msk) // from the reset pin
      ucResetMesg |= RESET_RST;

   pstTxMessage->ANT_MESSAGE_aucMesgData[0] = ucResetMesg;
   pstTxMessage->ANT_MESSAGE_ucSize         = MESG_STARTUP_MESG_SIZE;
   pstTxMessage->ANT_MESSAGE_ucMesgID       = MESG_STARTUP_MESG_ID;
}

/**
 * @brief Application system reset handler
 */
uint8_t System_Reset(uint8_t ucResetCmd)
{
   uint32_t ulErrorCode;

#if defined (COMPLETE_CHIP_SYSTEM_RESET)

   ulErrorCode = sd_power_reset_reason_clr( POWER_RESETREAS_OFF_Msk | POWER_RESETREAS_LOCKUP_Msk | POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_RESETPIN_Msk);
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_SystemReset();
   // should not return here, but in case it does catch it
   APP_ERROR_CHECK(ulErrorCode);

#else
   #error //unsupported reset scheme

#endif
   return NO_RESPONSE_MESSAGE;
}

/**
 * @brief Application deep sleep configuration handler
 */
static uint8_t ucDeepSleepFlag = 0;
uint8_t System_SetDeepSleep(ANT_MESSAGE *pstRxMessage)
{
   if (pstRxMessage->ANT_MESSAGE_ucSize == 0x01)
      ucDeepSleepFlag = 1;
   else
      return INVALID_MESSAGE;

   return RESPONSE_NO_ERROR;
}

/**
 * @brief Used for asynchronous serial suspend function
 */
void System_SetSuspendSleep(void)
{
  ucDeepSleepFlag = 1;
}

/**
 * @brief Application deep sleep handler
 */
void System_DeepSleep(void)
{
   uint32_t ulErrorCode;

   if (ucDeepSleepFlag)
   {
      ulErrorCode = sd_power_system_off(); // this should not return;
      APP_ERROR_CHECK(ulErrorCode);

      while(1); //reset is our only hope.
   }
}

#if !defined (SERIAL_NUMBER_NOT_AVAILABLE)
/**
 * @brief Constructs serial number message and calls function to retrieve serial number.
 */
void System_GetSerialNumberMesg(ANT_MESSAGE *pstTxMessage)
{
   pstTxMessage->ANT_MESSAGE_ucSize   = MESG_GET_SERIAL_NUM_SIZE;
   pstTxMessage->ANT_MESSAGE_ucMesgID = MESG_GET_SERIAL_NUM_ID;
   System_GetSerialNumber(pstTxMessage->ANT_MESSAGE_aucMesgData);
}

/**
 * @brief Retrieves lower 32-bits of 64-bit Nordic device ID stored in FICR. Returns ESN.
 */
void System_GetSerialNumber(uint8_t *pucESN)
{
   // read out permanent random device ID from FICR NVM
   uint32_t dev_id = NRF_FICR->DEVICEID[0];

   /* Allows us to read the permanent random device ID from FICR */
   //construct ESN to little endian 32-bit number
   pucESN[0] = dev_id >>  0 & 0x000000FF;
   pucESN[1] = dev_id >>  8 & 0x000000FF;
   pucESN[2] = dev_id >> 16 & 0x000000FF;
   pucESN[3] = dev_id >> 24 & 0x000000FF;
}
#endif // !SERIAL_NUMBER_NOT_AVAILABLE
