/*******************************************************************************
 *
 *  File:          lorawan-keys_example.h
 * 
 *  Function:      Example for lorawan-keys.h required by LMIC-node.
 *
 *  Copyright:     Copyright (c) 2021 Leonel Lopes Parente
 *
 *  Important      ██ DO NOT EDIT THIS EXAMPLE FILE (see instructions below) ██
 * 
 *  Decription:    lorawan-keys.h defines LoRaWAN keys needed by the LMIC library.
 *                 It can contain keys for both OTAA and for ABP activation.
 *                 Only the keys for the used activation type need to be specified.
 * 
 *                 It is essential that each key is specified in the correct format.
 *                 lsb: least-significant-byte first, msb: most-significant-byte first.
 * 
 *                 For security reasons all files in the keyfiles folder (except file
 *                 lorawan-keys_example.h) are excluded from the Git(Hub) repository.
 *                 Also excluded are all files matching the pattern *lorawan-keys.h.
 *                 This way they cannot be accidentally committed to a public repository.
 * 
 *  Instructions:  1. Copy this file lorawan-keys_example.h to file lorawan-keys.h
 *                    in the same folder (keyfiles).
 *                 2. Place/edit required LoRaWAN keys in the new lorawan-keys.h file.
 *
 ******************************************************************************/

#pragma once

#ifndef LORAWAN_KEYS_H_
#define LORAWAN_KEYS_H_

// Optional: If DEVICEID is defined it will be used instead of the default defined in the BSF.
// #define DEVICEID "<deviceid>"

// Keys required for OTAA activation: datalogger-v3-70b3d57ed0055fc7

// End-device Identifier (u1_t[8]) in lsb format
#define OTAA_DEVEUI 0xC7, 0x5F, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70

// Application Identifier (u1_t[8]) in lsb format
#define OTAA_APPEUI 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

// Application Key (u1_t[16]) in msb format
#define OTAA_APPKEY 0x96, 0x23, 0x72, 0xFF, 0x63, 0xC5, 0x1A, 0xFC, 0x7F, 0xF3, 0xB2, 0xB0, 0x3E, 0xFC, 0x72, 0xD7


// -----------------------------------------------------------------------------

// Optional: If ABP_DEVICEID is defined it will be used for ABP instead of the default defined in the BSF.
// #define ABP_DEVICEID "<deviceid>"

// Keys required for ABP activation:

// End-device Address (u4_t) in uint32_t format. 
// Note: The value must start with 0x (current version of TTN Console does not provide this).
#define ABP_DEVADDR 0x260B8B70

// Network Session Key (u1_t[16]) in msb format
#define ABP_NWKSKEY 0x5D, 0x5E, 0xEC, 0x4E, 0x92, 0x68, 0x2F, 0x76, 0xAF, 0x8C, 0x21, 0xF6, 0x9D, 0xF8, 0xD8, 0x8E

// Application Session K (u1_t[16]) in msb format
#define ABP_APPSKEY 0x74, 0x4C, 0x7E, 0x1E, 0xF6, 0x04, 0x1E, 0x07, 0x0D, 0xBD, 0xBC, 0x3D, 0x10, 0x3B, 0x67, 0x41


#endif  // LORAWAN_KEYS_H_
