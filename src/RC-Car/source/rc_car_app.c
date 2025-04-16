/******************************************************************************
* File Name:   udp_server.c
*
* Description: This file contains declaration of task and functions related to
*              UDP server operation.
*
********************************************************************************
* Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes */
#include "cy8c624abzi_s2d44.h"
#include "cy_crypto_server.h"
#include "cy_result.h"
#include "cy_syslib.h"
#include "cy_tcpwm.h"
#include "cy_tcpwm_counter.h"
#include "cy_tcpwm_pwm.h"
#include "cy_utils.h"
#include "cycfg_pins.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
// #include <cassert>
#include <inttypes.h>

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/_timeval.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

/* UDP server task header file. */
#include "rc_car_app.h"

#include "network_driver.h"
#include "portmacro.h"
#include "utils.h"
#include <sys/time.h>


#if defined (CY_USING_HAL)
    #include "cyhal_hwmgr.h"
#endif /* defined (CY_USING_HAL) */

#include "cycfg_peripherals.h"


#define UDP_SERVER_TASK_STACK_SIZE                (5 * 1024)
#define UDP_SERVER_TASK_PRIORITY                  (1)

#define TCPWM_SPEED_SENSOR                        (2UL)
#define TCPWM_SPEED_REFERENCE                     (3UL)

#define ULTRASONIC_CONVERSION                     (0.001379f)

#define NUM_REGS                                  (256)


/* Command definitions */
typedef enum __attribute__((__packed__)) 
{
    CMD_NOOP = 0x00,
    CMD_DIR,
    CMD_STEER,
    CMD_READ_REG,
} commands_t;

typedef struct {
    val_type_t reg_val;
    uint8_t rule;  // If True, then read only
} reg_map_t;


static float speed;
static volatile uint32_t capture;
static volatile uint32_t last_capture;

TaskHandle_t network_handle;

static reg_map_t register_map[NUM_REGS] = {
    {0, 0},  // Register 0
    {0, 0},  // Register 1
    {0, 0},  // Register 2
};

static bool process_command( client_req_t* req );
static void read_speed( void *arg );
static void encoder_pulse_handler( void *callback_arg, cyhal_gpio_event_t event );
static bool read_reg( REGISTERS_t reg, val_type_t* val );
static void send_reply( reply_t* reply );


/**
 * @brief Initialize the network driver
 * 
 * @return int Error code:
            - Failed to set the callback
            - Failed to start the network thread
 */
int rc_car_init( void )
{
    BaseType_t ret;

    // Start the frequency counter
    Cy_TCPWM_Counter_Enable(TCPWM0, TCPWM_SPEED_SENSOR);
    Cy_TCPWM_TriggerStart_Single(TCPWM0, TCPWM_SPEED_SENSOR);
    Cy_TCPWM_Counter_Init(TCPWM0, TCPWM_SPEED_SENSOR, &tcpwm_0_cnt_2_config);

    set_network_callback( process_command );  // Event based command processing
    ret = xTaskCreate(network_task, "Network task", UDP_SERVER_TASK_STACK_SIZE, NULL,
               UDP_SERVER_TASK_PRIORITY, &network_handle);
    if ( ret != pdPASS ) {
        return -1;
    }

    ret = xTaskCreate(rc_car_app_task, "RC task", UDP_SERVER_TASK_STACK_SIZE, NULL,
               UDP_SERVER_TASK_PRIORITY, &network_handle);
    if ( ret != pdPASS ) {
        return -1;
    }

    ret = xTaskCreate(read_speed, "speed reader", UDP_SERVER_TASK_STACK_SIZE, NULL,
               UDP_SERVER_TASK_PRIORITY, &network_handle);
    if ( ret != pdPASS ) {
        return -1;
    }

    return 0;
}


/*******************************************************************************
 * Function Name: rc_car_app_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote UDP client.
 *isr_button_press
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void rc_car_app_task(void *arg)
{
    /* Handle communication with the client */
    while (true)
    {
        speed = capture / 0.1;
        printf("Capture %.2f\r\n", speed);
        vTaskDelay( 50 );
    }
}


static void read_speed( void *arg ) {
    while( true ) {
        capture = Cy_TCPWM_Counter_GetCounter( TCPWM0, TCPWM_SPEED_SENSOR );

        // Reset the counter
        Cy_TCPWM_PWM_SetCounter( TCPWM0, TCPWM_SPEED_SENSOR, 0 );
        vTaskDelay( 100 );
    }
}


/**
 * @brief This function is passed as a handler to the network driver. It is to be called 
 * upon succesful reception of a UDP message.
 * 
 * @param req Pointer to request structure
 * @return true: Command processed successfully
 * @return false: Failed to process command
 */
static bool process_command( client_req_t* req ) {
    if ( req == NULL )
    {
        return false;
    }

    reply_t reply;

    switch ( req->payload.command )
    {
        case CMD_DIR   : printf("Direction command received: %i\r\n",   req->payload.data.i32); break;
        case CMD_STEER : printf("Steer command received: %i\r\n",       req->payload.data.i32); break;
        case CMD_READ_REG: {
            if (req->payload.data.u32 > REG_MAX) {
                reply.state = false;
            }

            reply.data.u32 = register_map[req->payload.data.u32].reg_val.u32;
            break;
        }
            
        default:         printf("ERROR: Command %u not recognized\r\n", req->payload.command ); return false;
    }

    send_reply( &reply );    
    return true;
}


static void send_reply( reply_t* reply ) {
    CY_ASSERT(reply != NULL);
    
    
}


/**
 * @brief Encoder pulse interrupt handler
 * 
 */
static void __attribute__((used)) encoder_pulse_handler( void *callback_arg, cyhal_gpio_event_t event ) {
    Cy_GPIO_ClearInterrupt(GPIO_PRT5, 0);
    NVIC_ClearPendingIRQ(ioss_interrupts_gpio_1_IRQn);

    // Grab capture from TCPWM
    capture = Cy_TCPWM_Counter_GetCapture(TCPWM0, TCPWM_SPEED_SENSOR);
}

/* [] END OF FILE */

