/*******************************************************************************
 * File Name: cycfg_routing.h
 *
 * Description:
 * Establishes all necessary connections between hardware elements.
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.40.0
 * device-db 4.20.0.7450
 * mtb-pdl-cat1 3.14.0.38372
 *
 *******************************************************************************
 * Copyright 2025 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#include "cycfg_notices.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define ioss_0_port_0_pin_0_ANALOG P0_0_SRSS_WCO_IN
#define ioss_0_port_0_pin_1_ANALOG P0_1_SRSS_WCO_OUT
#define ioss_0_port_0_pin_4_HSIOM P0_4_PERI_TR_IO_OUTPUT0
#define ioss_0_port_1_pin_0_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_5_pin_0_HSIOM P5_0_PERI_TR_IO_INPUT10
#define ioss_0_port_6_pin_4_HSIOM P6_4_CPUSS_SWJ_SWO_TDO
#define ioss_0_port_6_pin_6_HSIOM P6_6_CPUSS_SWJ_SWDIO_TMS
#define ioss_0_port_6_pin_7_HSIOM P6_7_CPUSS_SWJ_SWCLK_TCLK
#define ioss_0_port_7_pin_1_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_7_pin_2_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_7_pin_7_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_8_pin_1_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_8_pin_2_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_8_pin_3_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_8_pin_4_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_8_pin_5_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_8_pin_6_HSIOM HSIOM_SEL_AMUXA
#define ioss_0_port_8_pin_7_HSIOM HSIOM_SEL_AMUXA
#define CYBSP_DEBUG_UART_RX_digital_in_0_TRIGGER_IN TRIG_IN_MUX_2_HSIOM_TR_OUT10
#define CYBSP_I2S_MCLK_digital_in_0_TRIGGER_IN CYBSP_DEBUG_UART_RX_digital_in_0_TRIGGER_IN
#define CYBSP_SW2_digital_out_0_TRIGGER_OUT TRIG_OUT_MUX_4_HSIOM_TR_IO_OUTPUT0
#define CYBSP_USER_BTN1_digital_out_0_TRIGGER_OUT CYBSP_SW2_digital_out_0_TRIGGER_OUT
#define CYBSP_USER_BTN_digital_out_0_TRIGGER_OUT CYBSP_SW2_digital_out_0_TRIGGER_OUT
#define CYBSP_WIFI_HOST_WAKE_digital_out_0_TRIGGER_OUT CYBSP_SW2_digital_out_0_TRIGGER_OUT
#define tcpwm_0_cnt_0_tr_overflow_0_TRIGGER_IN TRIG_IN_MUX_2_TCPWM0_TR_OVERFLOW0
#define tcpwm_0_cnt_1_count_0_TRIGGER_OUT TRIG_OUT_MUX_2_TCPWM0_TR_IN8
#define tcpwm_0_cnt_2_capture_0_TRIGGER_OUT TRIG_OUT_MUX_2_TCPWM0_TR_IN9
#define tcpwm_0_cnt_2_count_0_TRIGGER_OUT TRIG_OUT_MUX_2_TCPWM0_TR_IN7
#define tcpwm_0_cnt_2_tr_overflow_0_TRIGGER_IN TRIG_IN_MUX_4_TCPWM0_TR_OVERFLOW2
#define tcpwm_0_cnt_3_tr_compare_match_0_TRIGGER_IN TRIG_IN_MUX_2_TCPWM0_TR_COMPARE_MATCH3
#define TCPWM0_CNT1_COUNT_VALUE 0xa
#define TCPWM0_CNT2_CAPTURE_VALUE 0xb
#define TCPWM0_CNT2_COUNT_VALUE 0x9

void init_cycfg_routing(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_ROUTING_H */
