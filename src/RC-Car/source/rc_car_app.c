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
#include "cy_utils.h"
#include "cycfg_pins.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
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


#define UDP_SERVER_TASK_STACK_SIZE                (5 * 1024)
#define UDP_SERVER_TASK_PRIORITY                  (1)

#define TCPWM_SPEED_SENSOR                        (2UL)
#define TCPWM_SPEED_REFERENCE                     (3UL)

#define ULTRASONIC_CONVERSION                     (0.001379f)


/* Command definitions */
typedef enum __attribute__((__packed__)) 
{
    CMD_NOOP = 0x00,
    CMD_DIR,
    CMD_STEER,
} commands_t;


TaskHandle_t network_handle;
static bool process_command( client_req_t* req );
static void read_speed( void );


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

    // Cy_TCPWM_Counter_Enable(TCPWM0, TCPWM_SPEED_SENSOR);
    // Cy_TCPWM_Counter_Enable(TCPWM0, TCPWM_SPEED_REFERENCE );

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
        read_speed();
        vTaskDelay( 1 );
    }
}


static void read_speed( void ) {
    struct timeval tv;
    xGetTimeTV(&tv);

    printf("Time: %lu:%lu\r\n", tv.tv_sec, tv.tv_usec);
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
        return -1;
    }

    switch ( req->payload.command )
    {
        case CMD_DIR   : printf("Direction command received: %i\r\n",   req->payload.data.i32); break;
        case CMD_STEER : printf("Steer command received: %i\r\n",       req->payload.data.i32); break;
        default:         printf("ERROR: Command %u not recognized\r\n", req->payload.command ); return false;
    }
    
    return 0;
}

/* [] END OF FILE */

