/*******************************************************************************
  Touch Library v3.16.0 Release

  Company:
    Microchip Technology Inc.

  File Name:
    touch.c

  Summary:
    QTouch Modular Library

  Description:
    Provides Initialization, Processing and ISR handler of touch library,
    Simple API functions to get/set the key touch parameters from/to the
    touch library data structures
*******************************************************************************/

/*******************************************************************************
Copyright (C) [2024], Microchip Technology Inc., and its subsidiaries. All rights reserved.

The software and documentation is provided by microchip and its contributors
"as is" and any express, implied or statutory warranties, including, but not
limited to, the implied warranties of merchantability, fitness for a particular
purpose and non-infringement of third party intellectual property rights are
disclaimed to the fullest extent permitted by law. In no event shall microchip
or its contributors be liable for any direct, indirect, incidental, special,
exemplary, or consequential damages (including, but not limited to, procurement
of substitute goods or services; loss of use, data, or profits; or business
interruption) however caused and on any theory of liability, whether in contract,
strict liability, or tort (including negligence or otherwise) arising in any way
out of the use of the software and documentation, even if advised of the
possibility of such damage.

Except as expressly permitted hereunder and subject to the applicable license terms
for any third-party software incorporated in the software and any applicable open
source software license terms, no license or other rights, whether express or
implied, are granted under any patent or other intellectual property rights of
Microchip or any third party.
************************************************************************************/



/*----------------------------------------------------------------------------
 *     include files
 *----------------------------------------------------------------------------*/
#include "definitions.h"
#include "../peripheral/rtc/plib_rtc.h"
#include "../interrupts.h"
#include "touch/touch.h"
#include "touch/touchTune.h"

/*----------------------------------------------------------------------------
 *   prototypes
 *----------------------------------------------------------------------------*/
/*! \brief configure keys, wheels and sliders.
 */
static touch_ret_t touch_sensors_config(void);

/*! \brief Touch measure complete callback function example prototype.
 */
static void qtm_measure_complete_callback(void);

/*! \brief Touch Error callback function prototype.
 */
static void qtm_error_callback(uint8_t error);



/*----------------------------------------------------------------------------
 *     Global Variables
 *----------------------------------------------------------------------------*/
/* Flag to indicate time for touch measurement */
static volatile uint8_t time_to_measure_touch_var = 0u;
/* post-process request flag */
static volatile uint8_t touch_postprocess_request = 0u;

/* Measurement Done Touch Flag  */
volatile uint8_t measurement_done_touch = 0u;

/* Error Handling */
uint8_t module_error_code = 0u;


/* Acquisition module internal data - Size to largest acquisition set */
static uint16_t touch_acq_signals_raw[DEF_NUM_CHANNELS];
/* Acquisition set 1 - General settings */
static qtm_acq_node_group_config_t ptc_qtlib_acq_gen1
    = {DEF_NUM_CHANNELS, DEF_SENSOR_TYPE, DEF_PTC_CAL_AUTO_TUNE, (uint8_t)DEF_SEL_FREQ_INIT, DEF_PTC_INTERRUPT_PRIORITY};

/* Node status, signal, calibration values */
qtm_acq_node_data_t ptc_qtlib_node_stat1[DEF_NUM_CHANNELS];

/* Node configurations */
qtm_acq_samd21_node_config_t ptc_seq_node_cfg1[DEF_NUM_CHANNELS] = {NODE_0_PARAMS,NODE_1_PARAMS,NODE_2_PARAMS,NODE_3_PARAMS,NODE_4_PARAMS,NODE_5_PARAMS,NODE_6_PARAMS,NODE_7_PARAMS,NODE_8_PARAMS,NODE_9_PARAMS,NODE_10_PARAMS,NODE_11_PARAMS,NODE_12_PARAMS,NODE_13_PARAMS,NODE_14_PARAMS,NODE_15_PARAMS,NODE_16_PARAMS,NODE_17_PARAMS,NODE_18_PARAMS,NODE_19_PARAMS,NODE_20_PARAMS,NODE_21_PARAMS,NODE_22_PARAMS,NODE_23_PARAMS,NODE_24_PARAMS,NODE_25_PARAMS,NODE_26_PARAMS,NODE_27_PARAMS,NODE_28_PARAMS,NODE_29_PARAMS,NODE_30_PARAMS,NODE_31_PARAMS,NODE_32_PARAMS,NODE_33_PARAMS,NODE_34_PARAMS,NODE_35_PARAMS,NODE_36_PARAMS,NODE_37_PARAMS,NODE_38_PARAMS,NODE_39_PARAMS,NODE_40_PARAMS,NODE_41_PARAMS,NODE_42_PARAMS,NODE_43_PARAMS,NODE_44_PARAMS,NODE_45_PARAMS,NODE_46_PARAMS,NODE_47_PARAMS,NODE_48_PARAMS,NODE_49_PARAMS,NODE_50_PARAMS,NODE_51_PARAMS,NODE_52_PARAMS,NODE_53_PARAMS,NODE_54_PARAMS,NODE_55_PARAMS,NODE_56_PARAMS,NODE_57_PARAMS,NODE_58_PARAMS,NODE_59_PARAMS,NODE_60_PARAMS,NODE_61_PARAMS,NODE_62_PARAMS,NODE_63_PARAMS,NODE_64_PARAMS,NODE_65_PARAMS,NODE_66_PARAMS,NODE_67_PARAMS,NODE_68_PARAMS,NODE_69_PARAMS,NODE_70_PARAMS,NODE_71_PARAMS,NODE_72_PARAMS,NODE_73_PARAMS,NODE_74_PARAMS,NODE_75_PARAMS,NODE_76_PARAMS,NODE_77_PARAMS,NODE_78_PARAMS,NODE_79_PARAMS,NODE_80_PARAMS,NODE_81_PARAMS,NODE_82_PARAMS,NODE_83_PARAMS,NODE_84_PARAMS,NODE_85_PARAMS,NODE_86_PARAMS,NODE_87_PARAMS,NODE_88_PARAMS,NODE_89_PARAMS,NODE_90_PARAMS,NODE_91_PARAMS,NODE_92_PARAMS,NODE_93_PARAMS,NODE_94_PARAMS,NODE_95_PARAMS,NODE_96_PARAMS,NODE_97_PARAMS,NODE_98_PARAMS,NODE_99_PARAMS,NODE_100_PARAMS,NODE_101_PARAMS,NODE_102_PARAMS,NODE_103_PARAMS,NODE_104_PARAMS,NODE_105_PARAMS,NODE_106_PARAMS,NODE_107_PARAMS,NODE_108_PARAMS,NODE_109_PARAMS,NODE_110_PARAMS,NODE_111_PARAMS,NODE_112_PARAMS,NODE_113_PARAMS,NODE_114_PARAMS,NODE_115_PARAMS,NODE_116_PARAMS,NODE_117_PARAMS,NODE_118_PARAMS,NODE_119_PARAMS,NODE_120_PARAMS,NODE_121_PARAMS,NODE_122_PARAMS,NODE_123_PARAMS,NODE_124_PARAMS,NODE_125_PARAMS,NODE_126_PARAMS,NODE_127_PARAMS};

/* Container */
static qtm_acquisition_control_t qtlib_acq_set1 = {&ptc_qtlib_acq_gen1, &ptc_seq_node_cfg1[0], &ptc_qtlib_node_stat1[0]};

/**********************************************************/
/*********** Frequency Hop Auto tune Module **********************/
/**********************************************************/

/* Buffer used with various noise filtering functions */
static uint16_t noise_filter_buffer[DEF_NUM_CHANNELS * NUM_FREQ_STEPS];
static uint8_t  freq_hop_delay_selection[NUM_FREQ_STEPS] = {DEF_MEDIAN_FILTER_FREQUENCIES};
static uint8_t  freq_hop_autotune_counters[NUM_FREQ_STEPS];

/* Configuration */
qtm_freq_hop_autotune_config_t qtm_freq_hop_autotune_config1 = {DEF_NUM_CHANNELS,
                                                                NUM_FREQ_STEPS,
                                                                &ptc_qtlib_acq_gen1.freq_option_select,
                                                                &freq_hop_delay_selection[0],
                                                                DEF_FREQ_AUTOTUNE_ENABLE,
                                                                FREQ_AUTOTUNE_MAX_VARIANCE,
                                                                FREQ_AUTOTUNE_COUNT_IN};

/* Data */
static qtm_freq_hop_autotune_data_t qtm_freq_hop_autotune_data1
    = {0, 0, &noise_filter_buffer[0], &ptc_qtlib_node_stat1[0], &freq_hop_autotune_counters[0]};

/* Container */
static qtm_freq_hop_autotune_control_t qtm_freq_hop_autotune_control1
    = {&qtm_freq_hop_autotune_data1, &qtm_freq_hop_autotune_config1};

/**********************************************************/
/*********************** Keys Module **********************/
/**********************************************************/

/* Keys set 1 - General settings */
qtm_touch_key_group_config_t qtlib_key_grp_config_set1 = {DEF_NUM_SENSORS,
                                                          DEF_TOUCH_DET_INT,
                                                          DEF_MAX_ON_DURATION,
                                                          DEF_ANTI_TCH_DET_INT,
                                                          DEF_ANTI_TCH_RECAL_THRSHLD,
                                                          DEF_TCH_DRIFT_RATE,
                                                          DEF_ANTI_TCH_DRIFT_RATE,
                                                          DEF_DRIFT_HOLD_TIME,
                                                          DEF_REBURST_MODE};

static qtm_touch_key_group_data_t qtlib_key_grp_data_set1;

/* Key data */
qtm_touch_key_data_t qtlib_key_data_set1[DEF_NUM_SENSORS];

/* Key Configurations */
qtm_touch_key_config_t qtlib_key_configs_set1[DEF_NUM_SENSORS] = { KEY_0_PARAMS, KEY_1_PARAMS, KEY_2_PARAMS, KEY_3_PARAMS, KEY_4_PARAMS, KEY_5_PARAMS, KEY_6_PARAMS, KEY_7_PARAMS, KEY_8_PARAMS, KEY_9_PARAMS, KEY_10_PARAMS, KEY_11_PARAMS, KEY_12_PARAMS, KEY_13_PARAMS, KEY_14_PARAMS, KEY_15_PARAMS, KEY_16_PARAMS, KEY_17_PARAMS, KEY_18_PARAMS, KEY_19_PARAMS, KEY_20_PARAMS, KEY_21_PARAMS, KEY_22_PARAMS, KEY_23_PARAMS, KEY_24_PARAMS, KEY_25_PARAMS, KEY_26_PARAMS, KEY_27_PARAMS, KEY_28_PARAMS, KEY_29_PARAMS, KEY_30_PARAMS, KEY_31_PARAMS, KEY_32_PARAMS, KEY_33_PARAMS, KEY_34_PARAMS, KEY_35_PARAMS, KEY_36_PARAMS, KEY_37_PARAMS, KEY_38_PARAMS, KEY_39_PARAMS, KEY_40_PARAMS, KEY_41_PARAMS, KEY_42_PARAMS, KEY_43_PARAMS, KEY_44_PARAMS, KEY_45_PARAMS, KEY_46_PARAMS, KEY_47_PARAMS, KEY_48_PARAMS, KEY_49_PARAMS, KEY_50_PARAMS, KEY_51_PARAMS, KEY_52_PARAMS, KEY_53_PARAMS, KEY_54_PARAMS, KEY_55_PARAMS, KEY_56_PARAMS, KEY_57_PARAMS, KEY_58_PARAMS, KEY_59_PARAMS, KEY_60_PARAMS, KEY_61_PARAMS, KEY_62_PARAMS, KEY_63_PARAMS, KEY_64_PARAMS, KEY_65_PARAMS, KEY_66_PARAMS, KEY_67_PARAMS, KEY_68_PARAMS, KEY_69_PARAMS, KEY_70_PARAMS, KEY_71_PARAMS, KEY_72_PARAMS, KEY_73_PARAMS, KEY_74_PARAMS, KEY_75_PARAMS, KEY_76_PARAMS, KEY_77_PARAMS, KEY_78_PARAMS, KEY_79_PARAMS, KEY_80_PARAMS, KEY_81_PARAMS, KEY_82_PARAMS, KEY_83_PARAMS, KEY_84_PARAMS, KEY_85_PARAMS, KEY_86_PARAMS, KEY_87_PARAMS, KEY_88_PARAMS, KEY_89_PARAMS, KEY_90_PARAMS, KEY_91_PARAMS, KEY_92_PARAMS, KEY_93_PARAMS, KEY_94_PARAMS, KEY_95_PARAMS, KEY_96_PARAMS, KEY_97_PARAMS, KEY_98_PARAMS, KEY_99_PARAMS, KEY_100_PARAMS, KEY_101_PARAMS, KEY_102_PARAMS, KEY_103_PARAMS, KEY_104_PARAMS, KEY_105_PARAMS, KEY_106_PARAMS, KEY_107_PARAMS, KEY_108_PARAMS, KEY_109_PARAMS, KEY_110_PARAMS, KEY_111_PARAMS, KEY_112_PARAMS, KEY_113_PARAMS, KEY_114_PARAMS, KEY_115_PARAMS, KEY_116_PARAMS, KEY_117_PARAMS, KEY_118_PARAMS, KEY_119_PARAMS, KEY_120_PARAMS, KEY_121_PARAMS, KEY_122_PARAMS, KEY_123_PARAMS, KEY_124_PARAMS, KEY_125_PARAMS, KEY_126_PARAMS,KEY_127_PARAMS}; 
/* Container */
static qtm_touch_key_control_t qtlib_key_set1
    = {&qtlib_key_grp_data_set1, &qtlib_key_grp_config_set1, &qtlib_key_data_set1[0], &qtlib_key_configs_set1[0]};




/*----------------------------------------------------------------------------
 *   function definitions
 *----------------------------------------------------------------------------*/


/*============================================================================
static touch_ret_t touch_sensors_config(void)
------------------------------------------------------------------------------
Purpose: Initialization of touch key sensors
Input  : none
Output : none
Notes  :
============================================================================*/
/* Touch sensors config - assign nodes to buttons / wheels / sliders / surfaces / water level / etc */
static touch_ret_t touch_sensors_config(void)
{
    uint16_t    sensor_nodes;
    touch_ret_t touch_ret = TOUCH_SUCCESS;

    /* Init acquisition module */
    touch_ret = qtm_ptc_init_acquisition_module(&qtlib_acq_set1);
    touch_ret = qtm_ptc_qtlib_assign_signal_memory(&touch_acq_signals_raw[0]);

    /* Initialize sensor nodes */
    for (sensor_nodes = 0u; sensor_nodes < (uint16_t) DEF_NUM_CHANNELS; sensor_nodes++) {
        /* Enable each node for measurement and mark for calibration */
        touch_ret = qtm_enable_sensor_node(&qtlib_acq_set1, sensor_nodes);
        touch_ret = qtm_calibrate_sensor_node(&qtlib_acq_set1, sensor_nodes);
    }

    /* Enable sensor keys and assign nodes */
    for (sensor_nodes = 0u; sensor_nodes < (uint16_t)DEF_NUM_SENSORS; sensor_nodes++) {
			touch_ret=qtm_init_sensor_key(&qtlib_key_set1, (uint8_t) sensor_nodes, &ptc_qtlib_node_stat1[sensor_nodes]);
    }



    return (touch_ret);
}

/*============================================================================
static void qtm_measure_complete_callback( void )
------------------------------------------------------------------------------
Purpose: Callback function called after the completion of
         measurement cycle. This function sets the post processing request
         flag to trigger the post processing.
Input  : none
Output : none
Notes  :
============================================================================*/
static void qtm_measure_complete_callback(void)
{
    touch_postprocess_request = 1u;
}

/*============================================================================
static void qtm_error_callback(uint8_t error)
------------------------------------------------------------------------------
Purpose: Callback function called after the completion of
         post processing. This function is called only when there is error.
Input  : error code
Output : decoded module error code
Notes  :
Derived Module_error_codes:
    Acquisition module error =1
    post processing module1 error = 2
    post processing module2 error = 3
    ... and so on

============================================================================*/
static void qtm_error_callback(uint8_t error)
{
	module_error_code = error + 1u;

}

/*============================================================================
void touch_init(void)
------------------------------------------------------------------------------
Purpose: Initialization of touch library. PTC, timer, and
         datastreamer modules are initialized in this function.
Input  : none
Output : none
Notes  :
============================================================================*/
void touch_init(void)
{
	touch_timer_config();

	/* Configure touch sensors with Application specific settings */
    (void)touch_sensors_config();

	
    #if DEF_TOUCH_TUNE_ENABLE == 1u
    touchTuneInit();
    #endif
}

/*============================================================================
void touch_process(void)
------------------------------------------------------------------------------
Purpose: Main processing function of touch library. This function initiates the
         acquisition, calls post processing after the acquistion complete and
         sets the flag for next measurement based on the sensor status.
Input  : none
Output : none
Notes  :
============================================================================*/
void touch_process(void)
{
    touch_ret_t touch_ret;

    /* check the time_to_measure_touch for Touch Acquisition */
    if (time_to_measure_touch_var == 1u) {

        /* Do the acquisition */
         touch_ret = qtm_ptc_start_measurement_seq(&qtlib_acq_set1, qtm_measure_complete_callback);

        /* if the Acquistion request was successful then clear the request flag */
        if (TOUCH_SUCCESS == touch_ret) {
            /* Clear the Measure request flag */
			time_to_measure_touch_var = 0;
        }
    }
    /* check the flag for node level post processing */
    if (touch_postprocess_request == 1u){
        /* Reset the flags for node_level_post_processing */
        touch_postprocess_request = 0u;
        /* Run Acquisition module level post processing*/
        touch_ret = qtm_acquisition_process();
        /* Check the return value */
        if (TOUCH_SUCCESS == touch_ret) {
            /* Returned with success: Start module level post processing */

            touch_ret = qtm_freq_hop_autotune(&qtm_freq_hop_autotune_control1);
            if (TOUCH_SUCCESS != touch_ret) {
                qtm_error_callback(1);
        }
            touch_ret = qtm_key_sensors_process(&qtlib_key_set1);
            if (TOUCH_SUCCESS != touch_ret) {
                qtm_error_callback(2);
            }
         }else {
           /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0);
        }


        #if DEF_TOUCH_TUNE_ENABLE == 1u
        touchTuneNewDataAvailable();
        #endif

        if (0u != (qtlib_key_set1.qtm_touch_key_group_data->qtm_keys_status & QTM_KEY_REBURST)) {
            time_to_measure_touch_var = 1u;
        } else {
            measurement_done_touch =1u;
        }
    }

    #if DEF_TOUCH_TUNE_ENABLE == 1u
    touchTuneProcess();
    #endif
}

/*============================================================================
void touch_timer_handler(void)
------------------------------------------------------------------------------
Purpose: This function updates the time elapsed to the touch key module to
         synchronize the internal time counts used by the module.
Input  : none
Output : none
Notes  :
============================================================================*/
void touch_timer_handler(void)
{
 
  
        time_to_measure_touch_var = 1u;
        qtm_update_qtlib_timer(DEF_TOUCH_MEASUREMENT_PERIOD_MS);
}
 
void rtc_cb( RTC_TIMER32_INT_MASK intCause, uintptr_t context ); 
void rtc_cb( RTC_TIMER32_INT_MASK intCause, uintptr_t context )
{
    touch_timer_handler();
}
static uintptr_t rtc_context;
void touch_timer_config(void)
{  
    RTC_Timer32CallbackRegister(rtc_cb, rtc_context);

    while((RTC_REGS->MODE0.RTC_STATUS & RTC_STATUS_SYNCBUSY_Msk) == RTC_STATUS_SYNCBUSY_Msk)
    {

    }
    /* Wait for Synchronization after writing value to Count Register */
    RTC_Timer32Stop();
    RTC_Timer32CounterSet(0u);

    RTC_Timer32CompareSet((uint32_t) DEF_TOUCH_MEASUREMENT_PERIOD_MS);
    RTC_Timer32Start();  
}

uint16_t get_sensor_node_signal(uint16_t sensor_node)
{
    return (ptc_qtlib_node_stat1[sensor_node].node_acq_signals);
}

void update_sensor_node_signal(uint16_t sensor_node, uint16_t new_signal)
{
    ptc_qtlib_node_stat1[sensor_node].node_acq_signals = new_signal;
}

uint16_t get_sensor_node_reference(uint16_t sensor_node)
{
    return (qtlib_key_data_set1[sensor_node].channel_reference);
}

void update_sensor_node_reference(uint16_t sensor_node, uint16_t new_reference)
{
    qtlib_key_data_set1[sensor_node].channel_reference = new_reference;
}

uint16_t get_sensor_cc_val(uint16_t sensor_node)
{
    return (ptc_qtlib_node_stat1[sensor_node].node_comp_caps);
}

void update_sensor_cc_val(uint16_t sensor_node, uint16_t new_cc_value)
{
    ptc_qtlib_node_stat1[sensor_node].node_comp_caps = new_cc_value;
}

uint8_t get_sensor_state(uint16_t sensor_node)
{
    return (qtlib_key_set1.qtm_touch_key_data[sensor_node].sensor_state);
}

void update_sensor_state(uint16_t sensor_node, uint8_t new_state)
{
    qtlib_key_set1.qtm_touch_key_data[sensor_node].sensor_state = new_state;
}

void calibrate_node(uint16_t sensor_node)
{
	touch_ret_t touch_ret = TOUCH_SUCCESS;
    /* Calibrate Node */
	touch_ret = qtm_calibrate_sensor_node(&qtlib_acq_set1, sensor_node);
    if(touch_ret != TOUCH_SUCCESS) {
		/* Error condition */
	}
    /* Initialize key */
    touch_ret = qtm_init_sensor_key(&qtlib_key_set1, (uint8_t) sensor_node, &ptc_qtlib_node_stat1[sensor_node]);
    if(touch_ret != TOUCH_SUCCESS) {
		/* Error condition */
	}
}



/*============================================================================
void PTC_Handler_EOC(void)
------------------------------------------------------------------------------
Purpose: Interrupt service handler for PTC EOC interrupt
Input  : none
Output : none
Notes  : none
============================================================================*/
void PTC_Handler(void)
{
	qtm_ptc_clear_interrupt();
	qtm_samd21_ptc_handler_eoc();
}
