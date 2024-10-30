/*******************************************************************************
  Touch Library v3.16.0 Release

  Company:
    Microchip Technology Inc.

  File Name:
    touch.h

  Summary:
    QTouch Modular Library

  Description:
    Configuration macros for touch library

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2024 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
#ifndef TOUCH_H
#define TOUCH_H
#include "device.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END
//SAMD21

/*----------------------------------------------------------------------------
 *     include files
 *----------------------------------------------------------------------------*/

#include "touch_api_ptc.h"

/**********************************************************/
/******************* Acquisition controls *****************/
/**********************************************************/
/* Defines the Measurement Time in milli seconds.
 * Range: 1 to 255.
 * Default value: 20.
 */
#define DEF_TOUCH_MEASUREMENT_PERIOD_MS 20u

/* Defines the Type of sensor
 * Default value: NODE_MUTUAL.
 */
#define DEF_SENSOR_TYPE NODE_MUTUAL

/* Set sensor calibration mode for charge share delay ,Prescaler or series resistor.
 * Range: CAL_AUTO_TUNE_NONE / CAL_AUTO_TUNE_RSEL / CAL_AUTO_TUNE_PRSC / CAL_AUTO_TUNE_CSD
 * Default value: CAL_AUTO_TUNE_NONE.
 */

#define DEF_PTC_CAL_OPTION   CAL_AUTO_TUNE_NONE

/* Calibration option to ensure full charge transfer */
/* Bits 7:0 = XX | TT SELECT_TAU | X | CAL_OPTION */
#define DEF_PTC_TAU_TARGET CAL_CHRG_5TAU
#define DEF_PTC_CAL_AUTO_TUNE (uint8_t)((DEF_PTC_TAU_TARGET << CAL_CHRG_TIME_POS) | DEF_PTC_CAL_OPTION)

/* Defines the interrupt priority for the PTC. Set low priority to PTC interrupt for applications having interrupt time
 * constraints.
 */
#define DEF_PTC_INTERRUPT_PRIORITY 3u

/* Set default bootup acquisition frequency.
 * Range: FREQ_SEL_0 - FREQ_SEL_15 , FREQ_SEL_SPREAD
 * Default value: FREQ_SEL_0.
 */
#define DEF_SEL_FREQ_INIT FREQ_SEL_0


/*----------------------------------------------------------------------------
 *     defines
 *----------------------------------------------------------------------------*/

/**********************************************************/
/***************** Node Params   ******************/
/**********************************************************/
/* Acquisition Set 1 */
/* Defines the number of sensor nodes in the acquisition set
 * Range: 1 to 65535.
 * Default value: 1
 */
#define DEF_NUM_CHANNELS (128u)


/* Defines node parameter setting
 * {X-line, Y-line, NODE_RSEL_PRSC(series resistor, prescaler), NODE_GAIN(Analog Gain , Digital Gain), filter level}
 */


#define NODE_0_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_1_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_2_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_3_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_4_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_5_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_6_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_7_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_8_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_9_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_10_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_11_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_12_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_13_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_14_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_15_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(15),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_16_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_17_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_18_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_19_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_20_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_21_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_22_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_23_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_24_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_25_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_26_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_27_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_28_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_29_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_30_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_31_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(14),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_32_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_33_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_34_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_35_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_36_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_37_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_38_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_39_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_40_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_41_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_42_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_43_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_44_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_45_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_46_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_47_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(13),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_48_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_49_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_50_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_51_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_52_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_53_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_54_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_55_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_56_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_57_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_58_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_59_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_60_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_61_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_62_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_63_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(12),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_64_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_65_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_66_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_67_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_68_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_69_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_70_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_71_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_72_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_73_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_74_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_75_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_76_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_77_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_78_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_79_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(0),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_80_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_81_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_82_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_83_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_84_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_85_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_86_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_87_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_88_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_89_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_90_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_91_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_92_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_93_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_94_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_95_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(1),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_96_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_97_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_98_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_99_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_100_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_101_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_102_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_103_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_104_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_105_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_106_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_107_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_108_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_109_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_110_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_111_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(10),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_112_PARAMS                                                                                               \
{                                                                                                                  \
   X(0), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_113_PARAMS                                                                                               \
{                                                                                                                  \
   X(1), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_114_PARAMS                                                                                               \
{                                                                                                                  \
   X(2), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_115_PARAMS                                                                                               \
{                                                                                                                  \
   X(3), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_116_PARAMS                                                                                               \
{                                                                                                                  \
   X(12), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_117_PARAMS                                                                                               \
{                                                                                                                  \
   X(13), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_118_PARAMS                                                                                               \
{                                                                                                                  \
   X(14), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_119_PARAMS                                                                                               \
{                                                                                                                  \
   X(15), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_120_PARAMS                                                                                               \
{                                                                                                                  \
   X(4), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_121_PARAMS                                                                                               \
{                                                                                                                  \
   X(5), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_122_PARAMS                                                                                               \
{                                                                                                                  \
   X(6), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_123_PARAMS                                                                                               \
{                                                                                                                  \
   X(7), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_124_PARAMS                                                                                               \
{                                                                                                                  \
   X(8), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_125_PARAMS                                                                                               \
{                                                                                                                  \
   X(9), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_126_PARAMS                                                                                               \
{                                                                                                                  \
   X(10), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}
#define NODE_127_PARAMS                                                                                               \
{                                                                                                                  \
   X(11), Y(11),  NODE_RSEL_PRSC(RSEL_VAL_0, (uint8_t)PRSC_DIV_SEL_4), NODE_GAIN(GAIN_1, GAIN_1), (uint8_t)FILTER_LEVEL_16                   \
}

/**********************************************************/
/***************** Key Params   ******************/
/**********************************************************/
/* Defines the number of key sensors
 * Range: 1 to 65535.
 * Default value: 1
 */
#define DEF_NUM_SENSORS (128u)

/* Defines Key Sensor setting
 * {Sensor Threshold, Sensor Hysterisis, Sensor AKS}
 */

#define KEY_0_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_1_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_2_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_3_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_4_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_5_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_6_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_7_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_8_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_9_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_10_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_11_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_12_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_13_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_14_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_15_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_16_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_17_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_18_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_19_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_20_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_21_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_22_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_23_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_24_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_25_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_26_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_27_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_28_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_29_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_30_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_31_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_32_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_33_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_34_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_35_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_36_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_37_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_38_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_39_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_40_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_41_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_42_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_43_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_44_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_45_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_46_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_47_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_48_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_49_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_50_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_51_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_52_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_53_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_54_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_55_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_56_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_57_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_58_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_59_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_60_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_61_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_62_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_63_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_64_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_65_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_66_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_67_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_68_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_69_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_70_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_71_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_72_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_73_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_74_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_75_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_76_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_77_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_78_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_79_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_80_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_81_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_82_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_83_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_84_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_85_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_86_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_87_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_88_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_89_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_90_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_91_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_92_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_93_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_94_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_95_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_96_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_97_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_98_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_99_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_100_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_101_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_102_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_103_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_104_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_105_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_106_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_107_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_108_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_109_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_110_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_111_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_112_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_113_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_114_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_115_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_116_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_117_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_118_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_119_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_120_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_121_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_122_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_123_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_124_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_125_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_126_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


#define KEY_127_PARAMS                                                                                            \
{                                                                                                              \
    20u, (uint8_t)HYST_25, (uint8_t)AKS_GROUP_1                       \
}


/* De-bounce counter for additional measurements to confirm touch detection
 * Range: 0 to 255.
 * Default value: 4.
 */
#define DEF_TOUCH_DET_INT 2u

/* De-bounce counter for additional measurements to confirm away from touch signal
 * to initiate Away from touch re-calibration.
 * Range: 0 to 255.
 * Default value: 5.
 */
#define DEF_ANTI_TCH_DET_INT 3u

/* Threshold beyond with automatic sensor recalibration is initiated.
 * Range: RECAL_100/ RECAL_50 / RECAL_25 / RECAL_12_5 / RECAL_6_25 / MAX_RECAL
 * Default value: RECAL_100.
 */
#define DEF_ANTI_TCH_RECAL_THRSHLD (uint8_t)RECAL_100

/* Rate at which sensor reference value is adjusted towards sensor signal value
 * when signal value is greater than reference.
 * Units: 200ms
 * Range: 0-255
 * Default value: 20u = 4 seconds.
 */
#define DEF_TCH_DRIFT_RATE 20u

/* Rate at which sensor reference value is adjusted towards sensor signal value
 * when signal value is less than reference.
 * Units: 200ms
 * Range: 0-255
 * Default value: 5u = 1 second.
 */
#define DEF_ANTI_TCH_DRIFT_RATE 20u

/* Time to restrict drift on all sensor when one or more sensors are activated.
 * Units: 200ms
 * Range: 0-255
 * Default value: 20u = 4 seconds.
 */
#define DEF_DRIFT_HOLD_TIME 20u

/* Set mode for additional sensor measurements based on touch activity.
 * Range: REBURST_NONE / REBURST_UNRESOLVED / REBURST_ALL
 * Default value: REBURST_UNRESOLVED
 */
#define DEF_REBURST_MODE (uint8_t)REBURST_UNRESOLVED

/* Sensor maximum ON duration upon touch.
 * Range: 0-255
 * Default value: 0
 */
#define DEF_MAX_ON_DURATION 5u




/**********************************************************/
/********* Frequency Hop Module ****************/
/**********************************************************/

/* sets the frequency steps for hop.
 * Range: 3 to 7.
 * Default value: 3
 */
#define NUM_FREQ_STEPS 3u

/* PTC Sampling Delay Selection - 0 to 15 PTC CLK cycles */

#define DEF_MEDIAN_FILTER_FREQUENCIES (uint8_t)FREQ_SEL_0,(uint8_t)FREQ_SEL_1,(uint8_t)FREQ_SEL_2

/* Enable / Disable the frequency hop auto tune
 * Range: 0 / 1
 * Default value: 1
 */
#define DEF_FREQ_AUTOTUNE_ENABLE 1u

/* sets the maximum variance for Frequency Hop Auto tune.
 * Range: 1 to 255.
 * Default value: 15
 */
#define FREQ_AUTOTUNE_MAX_VARIANCE 25u

/* sets the Tune in count for Frequency Hop Auto tune.
 * Range: 1 to 255.
 * Default value: 6
 */
#define FREQ_AUTOTUNE_COUNT_IN 6u

/**********************************************************/
/***************** Communication - Data Streamer ******************/
/**********************************************************/
#define DEF_TOUCH_DATA_STREAMER_ENABLE 0u

#define DEF_TOUCH_TUNE_ENABLE 1u


/**********************************************************/



/* Acquisition variables */
extern qtm_acq_node_data_t ptc_qtlib_node_stat1[DEF_NUM_CHANNELS];
extern qtm_acq_samd21_node_config_t ptc_seq_node_cfg1[DEF_NUM_CHANNELS];


/* Keys variables */
extern qtm_touch_key_group_config_t qtlib_key_grp_config_set1;
extern qtm_touch_key_data_t qtlib_key_data_set1[DEF_NUM_SENSORS];
extern qtm_touch_key_config_t qtlib_key_configs_set1[DEF_NUM_SENSORS];
/* Frequency Hop Autotune variables */
extern qtm_freq_hop_autotune_config_t qtm_freq_hop_autotune_config1;
extern uint8_t module_error_code;


extern volatile uint8_t measurement_done_touch;

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // TOUCH_H
