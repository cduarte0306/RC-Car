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
#include "cy_crypto_server.h"
#include "cy_result.h"
#include "cy_syslib.h"
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
#include <task.h>
#include <timers.h>
#include <queue.h>

/* Cypress secure socket header file */
#include "cy_secure_sockets.h"

/* Wi-Fi connection manager header files */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* UDP server task header file. */
#include "rc_car_app.h"

#include "cyhal_gpio.h"
#include "cyhal_psoc6_02_124_bga.h"
#include "netif.h"
#include "portmacro.h"
#include "projdefs.h"
#include "tcp.h"
#include "wifi-config.h"

#include "utils.h"

#include "lwip/udp.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* RTOS related macros for UDP server task. */
#define RTOS_TASK_TICKS_TO_WAIT                   (1000)
#define TIMER_TIMEOUT                             (1000)

/* Length of the LED ON/OFF command issued from the UDP server. */
#define UDP_LED_CMD_LEN                           (1)

/* LED ON and LED OFF commands. */
#define LED_ON_CMD                                '1'
#define LED_OFF_CMD                               '0'

#define LED_ON_ACK_MSG                            "LED ON ACK"

/* Initial message sent to UDP Server to confirm client availability. */
#define START_COMM_MSG                            'A'

/* Interrupt priority of the user button. */
#define USER_BTN_INTR_PRIORITY                    (5)

/* Buffer size to store the incoming messages from server, in bytes. */
#define MAX_UDP_RECV_BUFFER_SIZE                  (20)

#define QUEUE_SIZE                                (10u)

/* Error codes */
#define UDP_MESSAGE_SIZE_ERR                      (cy_rslt_t)0x00000001U
#define UDP_CRC_SIZE_ERR                          (cy_rslt_t)0x00000002U
#define UDP_MSG_ERR                               (cy_rslt_t)0x00000002U
#define UDP_REPLY_ERR                             (cy_rslt_t)0x00000003U

#define CY_SOCKET_INADDR_ANY (0x00000000)


/*******************************************************************************
Enums 
********************************************************************************/
/* Connection states */
typedef enum __attribute__((__packed__)) 
{
    DISCONNECTED = 0x00,
    CONNECTED
} wifi_connection_states_t;

/* Command definitions */
typedef enum __attribute__((__packed__)) 
{
    CMD_NOOP = 0x00,
    CMD_FWD_DIR,
    CMD_STEER,
} commands_t;

/*******************************************************************************
Protocol definitions 
********************************************************************************/
typedef struct __attribute__((__packed__))
{
    uint32_t   sequence_id;
    uint16_t   msg_length;

    struct __attribute__((__packed__))
    {
        uint8_t    command;
        val_type_t data;
    } payload;
    
    uint32_t   crc_32;
} client_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t  acknoledge;
    uint16_t message_length;
    
    struct __attribute__((__packed__))
    {
        uint32_t sequence_id;
        uint8    command;
    } payload;
    
    uint32   crc;
} server_ack_t;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static cy_rslt_t create_wifi_ap(void);
static cy_rslt_t create_udp_server_socket(void);
static cy_rslt_t udp_server_recv_handler(cy_socket_t socket_handle, void *arg);

static bool process_command( client_req_t* req );

static void timer_callback( TimerHandle_t xTimer );
static void isr_button_press( void *callback_arg, cyhal_gpio_event_t event);
void print_heap_usage(char *msg);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Secure socket variables. */
cy_socket_sockaddr_t udp_server_addr;
cy_socket_t server_handle;

cy_socket_sockaddr_t client_addr;
uint32_t client_addr_len = sizeof(client_addr);

wifi_connection_states_t connection_state = DISCONNECTED;

/* Connection timeout timer */
TimerHandle_t xTimer;
QueueHandle_t queue;

/*******************************************************************************
* Global Variables
********************************************************************************/
static const cy_wcm_ip_setting_t ap_sta_mode_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, SOFTAP_IP_ADDRESS),
    INITIALISER_IPV4_ADDRESS( .netmask,    SOFTAP_NETMASK),
    INITIALISER_IPV4_ADDRESS( .gateway,    SOFTAP_GATEWAY),
};


/* Flag variable to track client connection status,
 * set to True when START_COMM_MSG is received from client. */
bool client_connected = false;

/* Flags to tack the LED state and command. */
bool led_state = CYBSP_LED_STATE_OFF;

/* UDP Server task handle. */
extern TaskHandle_t server_task_handle;

cyhal_gpio_callback_data_t cb_data =
{
.callback = isr_button_press,
.callback_arg = NULL
};

/*******************************************************************************
 * Function Name: rc_car_app_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote UDP client.
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void rc_car_app_task(void *arg)
{
    cy_rslt_t result;
    bool ret;

    /* Variable to store number of bytes sent over UDP socket. */
    uint32_t bytes_sent = 0;

    /* Initialize the user button (CYBSP_USER_BTN) and register interrupt on falling edge. */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, USER_BTN_INTR_PRIORITY, true);

    cyhal_gpio_init(USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_BTN_OFF);

    cyhal_gpio_write( USER_LED, true );

    /* Create Wi-Fi access point. The client should be able to connect over Wi-Fi */
    if (create_wifi_ap() != CY_RSLT_SUCCESS)
    {
        printf("\n Failed to connect to Wi-Fi AP.\n");
        CY_ASSERT(0);
    }

    /* Create the timer */
    xTimer = xTimerCreate("Timer", TIMER_TIMEOUT, pdFALSE, (void *)0, timer_callback);
    if (xTimer == NULL)
    {
        CY_ASSERT(false);
    }

    ret = xTimerStart( xTimer, 1000 );
    if ( !ret ) {
        CY_ASSERT(false);
    }

    /* Create a queue */
    queue = xQueueCreate(QUEUE_SIZE, sizeof(server_ack_t));
    if (queue == NULL)
    {
        CY_ASSERT(false);
    }

    /* Secure Sockets initialization */
    result = cy_socket_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Secure Sockets initialization failed!\n");
        CY_ASSERT(0);
    }
    printf("Secure Sockets initialized\n");

    /* Create UDP Server */
    result = create_udp_server_socket();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("UDP Server Socket creation failed. Error: %" PRIu32 "\n", result);
        CY_ASSERT(0);
    }

    server_ack_t ack;

    /* Handle communication with the client */
    while (true)
    {
        xQueueReceive(queue, (void *const)&ack, portMAX_DELAY);

        result = cy_socket_sendto(server_handle, (void *const)&ack, sizeof(server_ack_t), CY_SOCKET_FLAGS_NONE,
                                  &client_addr, client_addr_len, &bytes_sent);

        if (result != CY_RSLT_SUCCESS)
        {
            printf("Failed to transmit to the client\r\n");
        }
    }
}


/**
 * @brief Create a wifi access point. The PSoC will host a Wifi access point, to which a computer will
 * have to connect in order to communicate with the device wirelessly.
 * 
 * @return cy_rslt_t 
 */
cy_rslt_t create_wifi_ap( void )
{
    cy_rslt_t result;
    cy_wcm_ap_config_t ap_conf;
    cy_wcm_ip_address_t ipv4_addr;

    cy_wcm_config_t wifi_config = {
            .interface = CY_WCM_INTERFACE_TYPE_AP_STA
    };

    /* Initialize Wi-Fi connection manager. */
    result = cy_wcm_init(&wifi_config);
    if ( result != CY_RSLT_SUCCESS ) {
        printf("Failed to setup WiFI AP\n");
    }

    memset(&ap_conf, 0, sizeof(cy_wcm_ap_config_t));
    memset(&ipv4_addr, 0, sizeof(cy_wcm_ip_address_t));

    ap_conf.channel = 1;
    memcpy(ap_conf.ap_credentials.SSID, SOFTAP_SSID, strlen(SOFTAP_SSID) + 1);
    memcpy(ap_conf.ap_credentials.password, SOFTAP_PASSWORD, strlen(SOFTAP_PASSWORD) + 1);
    
    ap_conf.ap_credentials.security = SOFTAP_SECURITY_TYPE;
    ap_conf.ip_settings.ip_address = ap_sta_mode_ip_settings.ip_address;
    ap_conf.ip_settings.netmask = ap_sta_mode_ip_settings.netmask;
    ap_conf.ip_settings.gateway = ap_sta_mode_ip_settings.gateway;

    result = cy_wcm_start_ap(&ap_conf);
    if ( result != CY_RSLT_SUCCESS ) {
        printf("Failed to setup WiFI AP\n");
    }
    
    /* Get IPV4 address for AP */
    result = cy_wcm_get_ip_addr(CY_WCM_INTERFACE_TYPE_AP, &ipv4_addr);

    return result;
}


/*******************************************************************************
 * Function Name: create_udp_server_socket
 *******************************************************************************
 * Summary:
 *  Function to create a socket and set the socket options
 *
 *******************************************************************************/
cy_rslt_t create_udp_server_socket(void)
{
    cy_rslt_t result;

    /* Variable used to set socket options. */
    cy_socket_opt_callback_t udp_recv_option = {
            .callback = udp_server_recv_handler,
            .arg = NULL
    };

    udp_server_addr.port = UDP_SERVER_PORT; // Your server port
    udp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;
    udp_server_addr.ip_address.ip.v4   = CY_SOCKET_INADDR_ANY;

    /* Create a UDP server socket. */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM, CY_SOCKET_IPPROTO_UDP, &server_handle);
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }

    /* Register the callback function to handle messages received from UDP client. */
    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
            CY_SOCKET_SO_RECEIVE_CALLBACK,
            &udp_recv_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }

    /* Bind the UDP socket created to Server IP address and port. */
    result = cy_socket_bind(server_handle, &udp_server_addr, sizeof(udp_server_addr));
    if (result == CY_RSLT_SUCCESS) {
        printf("Socket bound to port: %d\n", udp_server_addr.port);
    }

    return result;
}


/*******************************************************************************
 * Function Name: udp_server_recv_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle incoming  message from UDP client
 *
 *******************************************************************************/
cy_rslt_t udp_server_recv_handler(cy_socket_t socket_handle, void *arg) {
    cy_rslt_t result;

    /* Variable to store the number of bytes received. */
    uint32_t bytes_received = 0;
    client_req_t* req  = NULL;
    server_ack_t  reply;
    bool ret;

    /* Buffer to store data received from Client. */
    char message_buffer[MAX_UDP_RECV_BUFFER_SIZE] = {0};

    /* Here, we reset the timer since the connection is still alive */
    xTimerReset( xTimer, portMAX_DELAY );
    cyhal_gpio_write( USER_LED, false );

    /* Receive incoming message from UDP server. */
    result = cy_socket_recvfrom(server_handle, message_buffer, sizeof(client_req_t), CY_SOCKET_FLAGS_NONE,
                                    &client_addr, &client_addr_len, &bytes_received);

    if ( result != CY_RSLT_SUCCESS )
    {
        return result;
    }

    /* Parse the command */
    if ( bytes_received != sizeof( client_req_t ) ) {
        return UDP_MESSAGE_SIZE_ERR;
    }

    req = ( client_req_t* )( message_buffer );
    if ( req->msg_length != ( bytes_received - sizeof( uint32_t ) - sizeof( uint16_t ) - sizeof( uint32_t ) ) ) {
        return UDP_MESSAGE_SIZE_ERR;
    }

    const uint32_t crc = crc32(( char* )&req->payload, sizeof( req->payload ));
    if ( req->crc_32 != crc ) {
        return UDP_CRC_SIZE_ERR;
    }

    /* Process the incoming network command and submit a reply to the client */
    ret = process_command( req );
    if ( !ret ) {
        reply.payload.command     = 0;
        reply.payload.sequence_id = 0;
    } else {
        reply.payload.command     = req->payload.command;
        reply.payload.sequence_id = req->sequence_id;
    }

    /* Reply to the client */
    ret = xQueueSend(queue, ( const void * )&reply, portMAX_DELAY);
    if ( !ret ) {
        return UDP_REPLY_ERR;
    }

    return CY_RSLT_SUCCESS;
}


/**
 * @brief Process incoming UDP commands
 * 
 * @param req 
 * @return true 
 * @return false 
 */
static bool process_command( client_req_t* req ) {
    if ( req == NULL ) {
        return false;
    }

    switch ( req->payload.command ) {
        case CMD_FWD_DIR : break;
        case CMD_STEER   : break;
        default: return false;
    }
    
    return true;
}


/*******************************************************************************
 * Function Name: isr_button_press
 *******************************************************************************
 *
 * Summary:
 *  GPIO interrupt service routine. This function detects button presses and
 *  sets the command to be sent to UDP client.
 *
 * Parameters:
 *  void *callback_arg : pointer to the variable passed to the ISR
 *  cyhal_gpio_event_t event : GPIO event type
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void isr_button_press( void *callback_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Variable to hold the LED ON/OFF command to be sent to the UDP client. */
    uint32_t led_state_cmd;

    /* Set the command to be sent to UDP client. */
    if(led_state == CYBSP_LED_STATE_ON)
    {
        led_state_cmd = LED_OFF_CMD;
    }
    else
    {
        led_state_cmd = LED_ON_CMD;
    }

    /* Set the flag to send command to UDP client. */
    xTaskNotifyFromISR(server_task_handle, led_state_cmd,
                      eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void timer_callback( TimerHandle_t xTimer )
{
    cyhal_gpio_write( USER_LED, true );
    connection_state = DISCONNECTED;
}


/* [] END OF FILE */

