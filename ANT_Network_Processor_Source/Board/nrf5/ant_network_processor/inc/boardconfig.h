/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/

#ifndef BOARDCONFIG_H
#define BOARDCONFIG_H

#if defined (NRF51)

// Initial pin config
#define PIN_DIR_INIT                                  (0UL)
#define PIN_OUT_INIT                                  (0UL)

// Serial Communication Configuration
#define SERIAL_PIN_PORTSEL                            (0UL)   // serial mode detection. low =  Asynchronous high = Synchronous

// Async Serial
#if !defined (ASYNCHRONOUS_DISABLE)
   #define SERIAL_ASYNC_PIN_RTS                       (5UL)   // out
   #define SERIAL_ASYNC_PIN_TXD                       (15UL)  // out
   #define SERIAL_ASYNC_PIN_RXD                       (12UL)  // in
   #define SERIAL_ASYNC_PIN_SLEEP                     (2UL)   // in
   #define SERIAL_ASYNC_PIN_SUSPEND                   (23UL)  // in
   #define SERIAL_ASYNC_BR1                           (6UL)   // baud rate pin 1 selector
   #define SERIAL_ASYNC_BR2                           (24UL)  // baud rate pin 2 selector
   #define SERIAL_ASYNC_BR3                           (9UL)   // baud rate pin 3 selector
#endif //!ASYNCHRONOUS_DISABLE

// Sync Serial
#if !defined (SYNCHRONOUS_DISABLE)
   #define SERIAL_SYNC_PIN_SMSGRDY                    (2UL)   // in
   #define SERIAL_SYNC_PIN_SRDY                       (23UL)  // in
   #define SERIAL_SYNC_PIN_SEN                        (5UL)   // out
   #define SERIAL_SYNC_PIN_SIN                        (12UL)  // in
   #define SERIAL_SYNC_PIN_SOUT                       (15UL)  // out
   #define SERIAL_SYNC_PIN_SCLK                       (24UL)  // out
//   #define SERIAL_SYNC_PIN_SFLOW                      (6UL)   // in. Bit synchronous not supported
   #define SERIAL_SYNC_PIN_BR3                        (9UL)   // bit rate pin selector
#endif //!SYNCHRONOUS_DISABLE

#elif defined (NRF52)

// Initial pin config
#define PIN_DIR_INIT                                  (0UL)
#define PIN_OUT_INIT                                  (0UL)

// Serial Communication Configuration
#define SERIAL_PIN_PORTSEL                            (8UL)   // serial mode detection. low =  Asynchronous high = Synchronous

// Async Serial
#if !defined (ASYNCHRONOUS_DISABLE)
   #define SERIAL_ASYNC_PIN_RTS                       (12UL)  // out
   #define SERIAL_ASYNC_PIN_TXD                       (17UL)  // out
   #define SERIAL_ASYNC_PIN_RXD                       (16UL)  // in
   #define SERIAL_ASYNC_PIN_SLEEP                     (7UL)   // in
   #define SERIAL_ASYNC_PIN_SUSPEND                   (6UL)   // in
   #define SERIAL_ASYNC_BR1                           (15UL)  // baud rate pin 1 selector
   #define SERIAL_ASYNC_BR2                           (11UL)  // baud rate pin 2 selector
   #define SERIAL_ASYNC_BR3                           (14UL)  // baud rate pin 3 selector
#endif //!ASYNCHRONOUS_DISABLE

// Sync Serial
#if !defined (SYNCHRONOUS_DISABLE)
   #define SERIAL_SYNC_PIN_SMSGRDY                    (7UL)   // in
   #define SERIAL_SYNC_PIN_SRDY                       (6UL)   // in
   #define SERIAL_SYNC_PIN_SEN                        (12UL)  // out
   #define SERIAL_SYNC_PIN_SIN                        (16UL)  // in
   #define SERIAL_SYNC_PIN_SOUT                       (17UL)  // out
   #define SERIAL_SYNC_PIN_SCLK                       (11UL)  // out
//   #define SERIAL_SYNC_PIN_SFLOW                      (15UL)  // in. Bit synchronous not supported
   #define SERIAL_SYNC_PIN_BR3                        (14UL)  // bit rate pin selector
#endif //!SYNCHRONOUS_DISABLE

#endif

//Baudrate Capabilities
//#define BAUD1200_UNSUPPORTED
//#define BAUD2400_UNSUPPORTED
//#define BAUD4800_UNSUPPORTED
//#define BAUD9600_UNSUPPORTED
//#define BAUD19200_UNSUPPORTED
//#define BAUD38400_UNSUPPORTED
#define BAUD50000_UNSUPPORTED
//#define BAUD57600_UNSUPPORTED
//#define BAUD115200_UNSUPPORTED
//#define BAUD230400_UNSUPPORTED
//#define BAUD460800_UNSUPPORTED
//#define BAUD921600_UNSUPPORTED

//Bit rate Capabilities
//#define BIT_RATE_K500_UNSUPPORTED
//#define BIT_RATE_M1_UNSUPPORTED
//#define BIT_RATE_M2_UNSUPPORTED
//#define BIT_RATE_M4_UNSUPPORTED
//#define BIT_RATE_M8_UNSUPPORTED

#endif // BOARDCONFIG_H
