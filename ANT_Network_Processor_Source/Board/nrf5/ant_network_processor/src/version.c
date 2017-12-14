/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/

/*
 * NOTES:
 *
 * version "AAA#.##B##"
 *
 * SW_VER_MAJOR   - Increases on any released applicaion major feature update/changes or new features
 * SW_VER_MINOR   - Increases on any release application minor feature update i.e. Bug fixing and minor features.
 * SW_VER_PREFIX  - Is fixed on this firmware.
 * SW_VER_POSTFIX - Increases on any internal development builds. OR might be used for tagging special builds. OR might be used on branch build
  */

#if defined (NRF51)
   #define     SW_VER_MAJOR      "2." // version 2.x supports S210 v5 (AXX5.00B00) and S310 v3 (BAJ5.00B00)
   #define     SW_VER_MINOR      "02"

   #define     SW_VER_PREFIX     "BDD" // NRF51 as ANT Network processor
   #define     SW_VER_POSTFIX    "B00"
#elif defined (NRF52)
   #define     SW_VER_MAJOR      "1." // version 1.x support 212 (BHI1.00B02) and S332 (BHJ1.00B02)
   #define     SW_VER_MINOR      "00"

   #define     SW_VER_PREFIX     "BHF" // NRF52 as ANT Network processor
   #define     SW_VER_POSTFIX    "B00"
#else
   #error No version defined. See version.c.
#endif

/***************************************************************************
*/
const char acAppVersion[] = SW_VER_PREFIX SW_VER_MAJOR SW_VER_MINOR SW_VER_POSTFIX;  // Max 11 characters including null
