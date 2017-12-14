/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright � Dynastream Innovations Inc. 2014
All rights reserved.
*/

#include "global.h"

#include <stdbool.h>
#include <stdint.h>

#include "appconfig.h"
#include "ant_interface.h"
#include "ant_parameters.h"

/*
 * Global control flags: Must be accessed/changed atomically as they can be accessed by multiple contexts
 * If bitfields are used, the entire bitfield operation must be atomic!!
 */
volatile bool bEventRXSerialMessageProcess;
volatile bool bEventANTProcessStart;
volatile bool bEventANTProcess;
volatile bool bEventBurstMessageProcess;
volatile bool bEventStartupMessage;
volatile bool bEventSetBaudrate;
volatile uint8_t ucQueuedTxBurstChannel;

uint8_t ucBurstSequence;

ANT_ENABLE stANTChannelEnable;
__align(4) uint8_t aucANTChannelBlock[ANT_ENABLE_GET_REQUIRED_SPACE(ANT_STACK_TOTAL_CHANNELS_ALLOCATED_MAX, ANT_STACK_ENCRYPTED_CHANNELS_MAX, ANT_STACK_TX_BURST_QUEUE_SIZE_MAX)];
