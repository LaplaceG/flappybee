/**************************************************************************//**
 * @brief Background image for the weatherstation demo 
 * @version 3.20.5
*******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#ifndef BACKGROUND_H
#define BACKGROUND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BACKGROUND_XSIZE 384
#define BACKGROUND_SINGLE_XSIZE 128
#define BACKGROUND_YSIZE 128
extern const uint8_t background[BACKGROUND_XSIZE*16];
extern const uint8_t backgroundSingle[BACKGROUND_SINGLE_XSIZE*16];
extern uint8_t backgroundBee[BACKGROUND_SINGLE_XSIZE*16];

#ifdef __cplusplus
}
#endif

#endif
