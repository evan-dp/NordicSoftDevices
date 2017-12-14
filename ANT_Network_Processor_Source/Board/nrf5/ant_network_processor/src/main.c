/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
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


uint32_t ulErrorCode;
uint8_t ucEventChannel;
uint8_t ucEventType;
bool bAllowSleep = 0;
bool bAllowSerialSleep = 0;
ANT_MESSAGE *pstRxMessage;
ANT_MESSAGE *pstTxMessage;
uint8_t aucCapabilities[12]; // require 8, 4 extra padding just in case for future
#if defined (NRF52)
nrf_nvic_state_t nrf_nvic_state;
#endif

/**
 * @brief Handler for application asserts
 */
void assert_nrf_callback(uint16_t usLineNum, const uint8_t *pucFileName)
{
#if defined (RESET_ON_ASSERT_AND_FAULTS)
   NVIC_SystemReset();
#else
    while(1); // loop for debugging
#endif // RESET_ON_ASSERT_AND_FAULTS
}

/**
 * @brief Handler for softdevice asserts
 */
#if defined (NRF52)
void softdevice_assert_callback(uint32_t ulId, uint32_t ulPC, uint32_t ulInfo)
{
#if defined (RESET_ON_ASSERT_AND_FAULTS)
   NVIC_SystemReset();
#else
    while(1); // loop for debugging
#endif // RESET_ON_ASSERT_AND_FAULTS
}
#else
void softdevice_assert_callback(uint32_t ulPC, uint16_t usLineNum, const uint8_t *pucFileName)
{
   assert_nrf_callback(usLineNum, pucFileName);
}
#endif

/**
 * @brief Application error handler function
 */
void app_error_handler(uint32_t ulErrorCode, uint32_t ulLineNum, const uint8_t * pucFileName)
{
#if defined (RESET_ON_ASSERT_AND_FAULTS)
   NVIC_SystemReset();
#else
    while(1); // loop for debugging
#endif // RESET_ON_ASSERT_AND_FAULTS
}

/**
 * @brief Handler for hard faults
 */
void HardFault_Handler(uint32_t ulProgramCounter, uint32_t ulLinkRegister)
{
#if defined (RESET_ON_ASSERT_AND_FAULTS)
   NVIC_SystemReset();
#else
    while(1); // loop for debugging
#endif // RESET_ON_ASSERT_AND_FAULTS
}

/**
 * @brief Handler for SWI0 interrupts
 */
void SWI0_IRQHandler(void)
{
   // unused
}

/**
 * @brief Handler for radio notification interrupts (SWI1)
 */
void RADIO_NOTIFICATION_IRQHandler(void)
{
   // unused
}

/**
 * @brief Handler for protocol events & SOC event interrupts (SWI2)
 */
void SD_EVT_IRQHandler(void)
{
   uint32_t ulEvent;

   while (sd_evt_get(&ulEvent) != NRF_ERROR_NOT_FOUND) // read out SOC events
   {
   }

   bEventANTProcessStart = 1; // start ANT event handler to check if there are any ANT events
}

/**
 * @brief Handler for UART0 interrupts
 */
void UART0_IRQHandler(void)
{
   Serial_UART0_IRQHandler();
}

/**
 * @brief Handler for GPIOTE interrupts
 */
void GPIOTE_IRQHandler(void)
{
   Serial_GPIOTE_IRQHandler();
}

/**
 * @brief Set application queued burst mode
 */
void Main_SetQueuedBurst(void)
{
   bEventBurstMessageProcess = 1; // ANT burst message to process
}

/**
 * @brief Main
 */
int main()
{
#if defined (NRF52)
   nrf_clock_lf_cfg_t clock_source;
   memset(&nrf_nvic_state, 0, sizeof(nrf_nvic_state));
#endif
   /*** scatter file loading done by sd_softdevice_enable must be done first before any RAM access ***/
#if defined (NRF52)
   clock_source.source = NRF_CLOCK_LF_SRC_XTAL;
   clock_source.rc_ctiv = 0; //Must be 0 for XTAL
   clock_source.rc_temp_ctiv = 0; //Must be 0 for XTAL
   clock_source.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_50_PPM;

   ulErrorCode = sd_softdevice_enable(&clock_source, softdevice_assert_callback, ANT_LICENSE_KEY);
   APP_ERROR_CHECK(ulErrorCode);
#else
   ulErrorCode = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_50_PPM, softdevice_assert_callback);
   APP_ERROR_CHECK(ulErrorCode);
#endif

   // note: IRQ priorities must be set correctly before enabling them!
#if defined (NRF52)
   ulErrorCode = sd_nvic_SetPriority(SWI0_IRQn, 7);
#else
   ulErrorCode = sd_nvic_SetPriority(SWI0_IRQn, 3);
#endif
   APP_ERROR_CHECK(ulErrorCode);

#if defined (NRF52)
   ulErrorCode = sd_nvic_SetPriority(RADIO_NOTIFICATION_IRQn, 3); // SW1_IRQn
#else
   ulErrorCode = sd_nvic_SetPriority(RADIO_NOTIFICATION_IRQn, 1); // SW1_IRQn
#endif
   APP_ERROR_CHECK(ulErrorCode);

#if defined (NRF52)
   ulErrorCode = sd_nvic_SetPriority(SD_EVT_IRQn, 7); // SW2_IRQn
#else
   ulErrorCode = sd_nvic_SetPriority(SD_EVT_IRQn, 3); // SW2_IRQn
#endif
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_EnableIRQ(SWI0_IRQn);
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_EnableIRQ(RADIO_NOTIFICATION_IRQn); // SW1_IRQn
   APP_ERROR_CHECK(ulErrorCode);

   ulErrorCode = sd_nvic_EnableIRQ(SD_EVT_IRQn); // SW2_IRQn
   APP_ERROR_CHECK(ulErrorCode);

#if defined (SCALABLE_CHANNELS_DEFAULT)
   // Initialize ANT Channels to default
   stANTChannelEnable.ucTotalNumberOfChannels = ANT_STACK_TOTAL_CHANNELS_ALLOCATED_DEFAULT;
   stANTChannelEnable.ucNumberOfEncryptedChannels = ANT_STACK_ENCRYPTED_CHANNELS_DEFAULT;
   stANTChannelEnable.pucMemoryBlockStartLocation = aucANTChannelBlock;
   stANTChannelEnable.usMemoryBlockByteSize = ANT_ENABLE_GET_REQUIRED_SPACE(ANT_STACK_TOTAL_CHANNELS_ALLOCATED_DEFAULT, ANT_STACK_ENCRYPTED_CHANNELS_DEFAULT, ANT_STACK_TX_BURST_QUEUE_SIZE_DEFAULT);
   ulErrorCode = sd_ant_enable(&stANTChannelEnable);
   APP_ERROR_CHECK(ulErrorCode);
#endif // SCALABLE_CHANNELS_DEFAULT

   ulErrorCode = sd_ant_capabilities_get(aucCapabilities); // get num channels supported by stack
   APP_ERROR_CHECK(ulErrorCode);
   stANTChannelEnable.ucTotalNumberOfChannels = aucCapabilities[0];

   bEventRXSerialMessageProcess = 0;
   bEventANTProcessStart = 0;
   bEventANTProcess = 0;
   bEventBurstMessageProcess = 0;
   bEventStartupMessage = 0;
   ucQueuedTxBurstChannel = 0xFF; // invalid channel number
   bEventSetBaudrate = 0;
   ucBurstSequence = 0;

   System_Init();

   Serial_Init();
   pstRxMessage = Serial_GetRxMesgPtr();
   pstTxMessage = Serial_GetTxMesgPtr();
#if defined (SERIAL_REPORT_RESET_MESSAGE)
   bEventStartupMessage = 1;
   System_ResetMesg((ANT_MESSAGE *)pstTxMessage); // send reset message upon system startup
#endif // !SERIAL_REPORT_RESET_MESSAGE

   // reset power reset reason after constructing startup message
   ulErrorCode = sd_power_reset_reason_clr(POWER_RESETREAS_OFF_Msk | POWER_RESETREAS_LOCKUP_Msk | POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DOG_Msk | POWER_RESETREAS_RESETPIN_Msk); // clear reset reasons
   APP_ERROR_CHECK(ulErrorCode);
   // loop forever

   while (1)
   {
      bAllowSleep = 1;

      bAllowSerialSleep = Serial_RxMessage(); // poll for receive, check if serial interface can sleep

      if(bEventSetBaudrate)
      {
          bEventSetBaudrate = 0; // clear set baudrate event flag
          Serial_ActivateAsyncBaudrate();
      }
      else if (bEventStartupMessage) // send out startup serial message
      {
         bEventStartupMessage = 0;

         bAllowSleep = 0;
      }
      else if (bEventRXSerialMessageProcess) // rx serial message to handle
      {
         bEventRXSerialMessageProcess = 0; // clear the RX event flag
         Command_SerialMessageProcess((ANT_MESSAGE *)pstRxMessage, (ANT_MESSAGE *)pstTxMessage); // send to command handler

         bAllowSleep = 0;
      }
      else if (bEventANTProcessStart || bEventANTProcess) // protocol event message to handle
      {
         if (!bEventANTProcess)
         {
            bEventANTProcessStart = 0;
            bEventANTProcess = 1;
         }

         ulErrorCode = sd_ant_event_get(&ucEventChannel, &ucEventType, (uint8_t*)pstTxMessage); // read event from ANT stack

         if ((ulErrorCode == NRF_SUCCESS) && ucEventType) // received event, send to event handlers
         {
            Serial_ANTEventHandler(ucEventType, pstTxMessage);
         }
         else // no event
         {
            bEventANTProcess = 0;
         }

         bAllowSleep = 0;
      }
      else if (bEventBurstMessageProcess) // we have a burst message to process
      {
         bEventBurstMessageProcess = 0; // clear the burst event flag
         if (Command_BurstMessageProcess((ANT_MESSAGE *)pstRxMessage, (ANT_MESSAGE *)pstTxMessage)) // try to process the burst transfer message
         {
            ucQueuedTxBurstChannel = ((ANT_MESSAGE *)pstRxMessage)->ANT_MESSAGE_ucChannel & CHANNEL_NUMBER_MASK; // indicate queued burst transfer process
         }

         bAllowSleep = 0;
      }

      Serial_TxMessage(); // transmit any pending tx messages, goes to sleep if it can

      if (bAllowSleep) // if sleep is allowed
      {
         if (bAllowSerialSleep)
            Serial_Sleep(); // serial interface sleep

         System_DeepSleep(); // goto deep sleep/system off if required

         (void)sd_app_evt_wait(); // wait for event low power mode
      }
   }
}
