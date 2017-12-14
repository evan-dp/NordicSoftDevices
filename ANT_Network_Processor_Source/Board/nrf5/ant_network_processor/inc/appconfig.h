/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/

#ifndef APPCONFIG_H
#define APPCONFIG_H

// Pre-allocated maximum memory allocation block for scalable channel support
#define ANT_STACK_TOTAL_CHANNELS_ALLOCATED_MAX        15   // Max number of channels supported by ANT stack (SIZE_OF_NONENCRYPTED_ANT_CHANNEL bytes each)
#define ANT_STACK_ENCRYPTED_CHANNELS_MAX              15   // Max number of encrypted channels supported by ANT stack (SIZE_OF_ENCRYPTED_ANT_CHANNEL bytes each)
#define ANT_STACK_TX_BURST_QUEUE_SIZE_MAX            128   // Max burst queue size allocated to ANT stack (bytes)

// Default SCALABLE_CHANNELS_DEFAULT settings
#define ANT_STACK_TOTAL_CHANNELS_ALLOCATED_DEFAULT    15   // Default number of channels supported by ANT stack (SIZE_OF_NONENCRYPTED_ANT_CHANNEL bytes each)
#define ANT_STACK_ENCRYPTED_CHANNELS_DEFAULT          15   // Default number of encrypted channels supported by ANT stack (SIZE_OF_ENCRYPTED_ANT_CHANNEL bytes each)
#define ANT_STACK_TX_BURST_QUEUE_SIZE_DEFAULT        128   // Default burst queue size allocated to ANT stack (bytes)

#define COMPLETE_CHIP_SYSTEM_RESET                         // ANT reset message causes NRF51 hard reset
#define SYSTEM_SLEEP                                       // Enable deep sleep command
#define SERIAL_REPORT_RESET_MESSAGE                        // Generate startup message

#define SCALABLE_CHANNELS_DEFAULT                          // Use SCALABLE_CHANNELS default settings

#endif // APPCONFIG_H
