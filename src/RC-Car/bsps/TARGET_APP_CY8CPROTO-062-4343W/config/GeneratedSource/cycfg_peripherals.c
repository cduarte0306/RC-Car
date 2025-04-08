/*******************************************************************************
 * File Name: cycfg_peripherals.c
 *
 * Description:
 * Peripheral Hardware Block configuration
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

#include "cycfg_peripherals.h"

#define tcpwm_0_cnt_0_INPUT_DISABLED 0x7U
#define tcpwm_0_cnt_1_INPUT_DISABLED 0x7U

cy_stc_csd_context_t cy_csd_0_context =
{
    .lockKey = CY_CSD_NONE_KEY,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t lcd_0_drive_0_obj =
{
    .type = CYHAL_RSC_LCD,
    .block_num = 0U,
    .channel_num = 0,
};
#endif /* defined (CY_USING_HAL) */

cy_stc_rtc_config_t srss_0_rtc_0_config =
{
    .sec = 0U,
    .min = 0U,
    .hour = 12U,
    .amPm = CY_RTC_AM,
    .hrFormat = CY_RTC_24_HOURS,
    .dayOfWeek = CY_RTC_SATURDAY,
    .date = 1U,
    .month = CY_RTC_JANUARY,
    .year = 0U,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t srss_0_rtc_0_obj =
{
    .type = CYHAL_RSC_RTC,
    .block_num = 0U,
    .channel_num = 0U,
};
cyhal_rtc_configurator_t srss_0_rtc_0_hal_config =
{
    .resource = &srss_0_rtc_0_obj,
    .config = &srss_0_rtc_0_config,
    .dst_config = NULL,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_tcpwm_counter_config_t tcpwm_0_cnt_0_config =
{
    .period = 1000000,
    .clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
    .runMode = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
    .compareOrCapture = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0 = 16384,
    .compare1 = 16384,
    .enableCompareSwap = false,
    .interruptSources = CY_TCPWM_INT_NONE,
    .captureInputMode = tcpwm_0_cnt_0_INPUT_DISABLED & 0x3U,
    .captureInput = CY_TCPWM_INPUT_0,
    .reloadInputMode = tcpwm_0_cnt_0_INPUT_DISABLED & 0x3U,
    .reloadInput = CY_TCPWM_INPUT_0,
    .startInputMode = tcpwm_0_cnt_0_INPUT_DISABLED & 0x3U,
    .startInput = CY_TCPWM_INPUT_0,
    .stopInputMode = tcpwm_0_cnt_0_INPUT_DISABLED & 0x3U,
    .stopInput = CY_TCPWM_INPUT_0,
    .countInputMode = tcpwm_0_cnt_0_INPUT_DISABLED & 0x3U,
    .countInput = CY_TCPWM_INPUT_1,
};

#if defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE)
const cyhal_resource_inst_t tcpwm_0_cnt_0_obj =
{
    .type = CYHAL_RSC_TCPWM,
    .block_num = 0U,
    .channel_num = 0U,
};
#endif /* defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE) */

#if defined(CY_USING_HAL_LITE) || defined (CY_USING_HAL)
const cyhal_clock_t tcpwm_0_cnt_0_clock =
{
    .block = CYHAL_CLOCK_BLOCK_PERIPHERAL_24_5BIT,
    .channel = 0,
#if defined (CY_USING_HAL)
    .reserved = false,
    .funcs = NULL,
#endif /* defined (CY_USING_HAL) */
};
#endif /* defined(CY_USING_HAL_LITE) || defined (CY_USING_HAL) */

#if defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE)
const cyhal_timer_configurator_t tcpwm_0_cnt_0_hal_config =
{
    .resource = &tcpwm_0_cnt_0_obj,
    .config = &tcpwm_0_cnt_0_config,
    .clock = &tcpwm_0_cnt_0_clock,
};
#endif /* defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE) */

#if defined (COMPONENT_MTB_HAL)
const mtb_hal_peri_div_t tcpwm_0_cnt_0_clock_ref =
{
    .clk_dst = (en_clk_dst_t)PCLK_TCPWM0_CLOCKS0,
    .div_type = CY_SYSCLK_DIV_24_5_BIT,
    .div_num = 0,
};
const mtb_hal_clock_t tcpwm_0_cnt_0_hal_clock =
{
    .clock_ref = &tcpwm_0_cnt_0_clock_ref,
    .interface = &mtb_hal_clock_peri_interface,
};
#endif /* defined (COMPONENT_MTB_HAL) */

#if defined (COMPONENT_MTB_HAL) && (MTB_HAL_DRIVER_AVAILABLE_TIMER)
const mtb_hal_timer_configurator_t tcpwm_0_cnt_0_hal_config =
{
    .tcpwm_base = tcpwm_0_cnt_0_HW,
    .clock = &tcpwm_0_cnt_0_hal_clock,
    .tcpwm_cntnum = 0U,
};
#endif /* defined (COMPONENT_MTB_HAL) && (MTB_HAL_DRIVER_AVAILABLE_TIMER) */

const cy_stc_tcpwm_counter_config_t tcpwm_0_cnt_1_config =
{
    .period = 0xFFFFFFFF,
    .clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
    .runMode = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
    .compareOrCapture = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .compare0 = 16384,
    .compare1 = 16384,
    .enableCompareSwap = false,
    .interruptSources = CY_TCPWM_INT_NONE,
    .captureInputMode = tcpwm_0_cnt_1_INPUT_DISABLED & 0x3U,
    .captureInput = CY_TCPWM_INPUT_0,
    .reloadInputMode = tcpwm_0_cnt_1_INPUT_DISABLED & 0x3U,
    .reloadInput = CY_TCPWM_INPUT_0,
    .startInputMode = tcpwm_0_cnt_1_INPUT_DISABLED & 0x3U,
    .startInput = CY_TCPWM_INPUT_0,
    .stopInputMode = tcpwm_0_cnt_1_INPUT_DISABLED & 0x3U,
    .stopInput = CY_TCPWM_INPUT_0,
    .countInputMode = CY_TCPWM_INPUT_RISINGEDGE,
    .countInput = TCPWM0_CNT1_COUNT_VALUE,
};

#if defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE)
const cyhal_resource_inst_t tcpwm_0_cnt_1_obj =
{
    .type = CYHAL_RSC_TCPWM,
    .block_num = 0U,
    .channel_num = 1U,
};
#endif /* defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE) */

#if defined(CY_USING_HAL_LITE) || defined (CY_USING_HAL)
const cyhal_clock_t tcpwm_0_cnt_1_clock =
{
    .block = CYHAL_CLOCK_BLOCK_PERIPHERAL_24_5BIT,
    .channel = 0,
#if defined (CY_USING_HAL)
    .reserved = false,
    .funcs = NULL,
#endif /* defined (CY_USING_HAL) */
};
#endif /* defined(CY_USING_HAL_LITE) || defined (CY_USING_HAL) */

#if defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE)
const cyhal_timer_configurator_t tcpwm_0_cnt_1_hal_config =
{
    .resource = &tcpwm_0_cnt_1_obj,
    .config = &tcpwm_0_cnt_1_config,
    .clock = &tcpwm_0_cnt_1_clock,
};
#endif /* defined (CY_USING_HAL) || defined(CY_USING_HAL_LITE) */

#if defined (COMPONENT_MTB_HAL)
const mtb_hal_peri_div_t tcpwm_0_cnt_1_clock_ref =
{
    .clk_dst = (en_clk_dst_t)PCLK_TCPWM0_CLOCKS1,
    .div_type = CY_SYSCLK_DIV_24_5_BIT,
    .div_num = 0,
};
const mtb_hal_clock_t tcpwm_0_cnt_1_hal_clock =
{
    .clock_ref = &tcpwm_0_cnt_1_clock_ref,
    .interface = &mtb_hal_clock_peri_interface,
};
#endif /* defined (COMPONENT_MTB_HAL) */

#if defined (COMPONENT_MTB_HAL) && (MTB_HAL_DRIVER_AVAILABLE_TIMER)
const mtb_hal_timer_configurator_t tcpwm_0_cnt_1_hal_config =
{
    .tcpwm_base = tcpwm_0_cnt_1_HW,
    .clock = &tcpwm_0_cnt_1_hal_clock,
    .tcpwm_cntnum = 1U,
};
#endif /* defined (COMPONENT_MTB_HAL) && (MTB_HAL_DRIVER_AVAILABLE_TIMER) */

void init_cycfg_peripherals(void)
{
    Cy_SysClk_PeriphAssignDivider(PCLK_CSD_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS0, CY_SYSCLK_DIV_24_5_BIT, 0U);
    Cy_SysClk_PeriphAssignDivider(PCLK_TCPWM0_CLOCKS1, CY_SYSCLK_DIV_24_5_BIT, 0U);
}
void reserve_cycfg_peripherals(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&lcd_0_drive_0_obj);
    cyhal_hwmgr_reserve(&srss_0_rtc_0_obj);
    cyhal_hwmgr_reserve(&tcpwm_0_cnt_0_obj);
    cyhal_hwmgr_reserve(&tcpwm_0_cnt_1_obj);
#endif /* defined (CY_USING_HAL) */
}
