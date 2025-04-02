/******************************************************************************
 * File Name:   udp_server.c
 *
 * Description: This file contains declaration of task and functions related to
 *              UDP server operation.
 *
 ********************************************************************************
 * Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company)
 *or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
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
#include "cy_retarget_io.h"
#include "cy_syslib.h"
#include "cy_utils.h"
#include "cyabs_rtos_impl.h"
#include "cybsp.h"
#include "cycfg_pins.h"
#include "cyhal.h"
#include <inttypes.h>

#include "cyhal_rtc.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <queue.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <task.h>
#include <timers.h>

/* Cypress secure socket header file */
#include "cy_secure_sockets.h"

/* Wi-Fi connection manager header files */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* UDP server task header file. */
#include "network_driver.h"

#include "cyhal_gpio.h"
#include "cyhal_psoc6_02_124_bga.h"
#include "def.h"
#include "netif.h"
#include "portmacro.h"
#include "projdefs.h"
#include "tcp.h"
#include "utils.h"
#include "wifi-config.h"

#include "lwip/ip_addr.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
/* RTOS related macros for UDP server task. */
#define RTOS_TASK_TICKS_TO_WAIT (1000)
#define TIMER_TIMEOUT (1000)

/* Length of the LED ON/OFF command issued from the UDP server. */
#define UDP_LED_CMD_LEN (1)

/* LED ON and LED OFF commands. */
#define LED_ON_CMD '1'
#define LED_OFF_CMD '0'

#define LED_ON_ACK_MSG "LED ON ACK"

/* Initial message sent to UDP Server to confirm client availability. */
#define START_COMM_MSG 'A'

/* Interrupt priority of the user button. */
#define USER_BTN_INTR_PRIORITY (5)

/* Buffer size to store the incoming messages from server, in bytes. */
#define MAX_UDP_RECV_BUFFER_SIZE (20)

#define QUEUE_SIZE (10u)

#define NTP_CLIENT_PORT (123)

/* Error codes */
#define UDP_MESSAGE_SIZE_ERR (cy_rslt_t)0x00000001U
#define UDP_CRC_SIZE_ERR (cy_rslt_t)0x00000002U
#define UDP_MSG_ERR (cy_rslt_t)0x00000002U
#define UDP_REPLY_ERR (cy_rslt_t)0x00000003U

#define CY_SOCKET_INADDR_ANY (0x00000000)

#define NTP_SERVER_TASK_STACK_SIZE                (1 * 1024)

#define NTP_SERVER_TASK_PRIORITY                  (1)
#define MICROSECONDS_TO_FRACTION( x ) (uint32_t)((uint64_t)(x) * 4294967296ULL / 1000000)


/*******************************************************************************
Enums
********************************************************************************/
/* Connection states */
typedef enum __attribute__((__packed__)) {
  DISCONNECTED = 0x00,
  CONNECTED
} wifi_connection_states_t;

/* Command definitions */
typedef enum __attribute__((__packed__)) {
  CMD_NOOP = 0x00,
  CMD_DIR,
  CMD_STEER,
} commands_t;

typedef enum {
    NTP_WAIT_FOR_ARP,
    NTP_REQUEST,
    NTP_REPLY,
    NTP_SLEEP
} ntp_states_t;

typedef struct __attribute__((__packed__)) {
  uint8_t acknowledge;
  uint16_t message_length;

  struct __attribute__((__packed__)) {
    uint32_t sequence_id;
    uint8 command;
  } payload;

  uint32 crc;
} server_ack_t;

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static cy_rslt_t create_wifi_ap(void);
static cy_rslt_t create_udp_server_socket(void);
static cy_rslt_t udp_server_recv_handler(cy_socket_t socket_handle, void *arg);
static cy_rslt_t ntp_receive_handler(cy_socket_t socket_handle, void *arg);
static void wifi_event_callback(cy_wcm_event_t event,
                                cy_wcm_event_data_t *event_data);

static void isr_button_press(void *callback_arg, cyhal_gpio_event_t event);
static void ntp_thread( void* arg );

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* Secure socket variables. */
static cy_socket_sockaddr_t udp_server_addr;
static cy_socket_sockaddr_t ntp_server_addr;

static cy_socket_t server_handle;
static cy_socket_t ntp_client_handle;

/* NTP socket handle */
static cy_socket_t ntp_socket_handle;

static cy_socket_sockaddr_t client_addr;
static uint32_t client_addr_len = sizeof(client_addr);

static wifi_connection_states_t connection_state = DISCONNECTED;

/* Connection timeout timer */
static QueueHandle_t queue;
static message_reception_callback udp_reply_callback = NULL;
static message_reception_callback ntp_reply_callback = NULL;

TaskHandle_t ntp_handle     = NULL;
ntp_states_t ntp_state = NTP_REQUEST;

ntp_packet_t ntp;



/*******************************************************************************
 * Global Variables
 ********************************************************************************/
static const cy_wcm_ip_setting_t ap_sta_mode_ip_settings = {
    INITIALISER_IPV4_ADDRESS(.ip_address, SOFTAP_IP_ADDRESS),
    INITIALISER_IPV4_ADDRESS(.netmask, SOFTAP_NETMASK),
    INITIALISER_IPV4_ADDRESS(.gateway, SOFTAP_GATEWAY),
};

/* Flag variable to track client connection status,
 * set to True when START_COMM_MSG is received from client. */
bool client_connected = false;

/* Flags to tack the LED state and command. */
bool led_state = CYBSP_LED_STATE_OFF;

/* UDP Server task handle. */
extern TaskHandle_t server_task_handle;

cyhal_gpio_callback_data_t cb_data = {.callback = isr_button_press,
                                      .callback_arg = NULL};

/**
 * @brief Set the callback for network receptions
 *
 * @param _callback Pointer to the callback
 */
void set_network_callback(network_callbacks_t _callback) {
    udp_reply_callback = _callback.command_received;
    ntp_reply_callback = _callback.ntp_reply_received;
}

/*******************************************************************************
 * Function Name: network_task
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
void network_task(void *arg) {
    CY_ASSERT( udp_reply_callback != NULL);

    cy_rslt_t result;

    memset( &ntp, 0, sizeof( ntp_packet_t ));

    /* Variable to store number of bytes sent over UDP socket. */
    uint32_t bytes_sent = 0;

    /* Initialize the user button (CYBSP_USER_BTN) and register interrupt on
    * falling edge. */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP,
                    CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            USER_BTN_INTR_PRIORITY, true);

    cyhal_gpio_init(USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG,
                    CYBSP_BTN_OFF);

    cyhal_gpio_write(USER_LED, true);

    /* Create Wi-Fi access point. The client should be able to connect over Wi-Fi
    */
    if (create_wifi_ap() != CY_RSLT_SUCCESS) {
    printf("\n Failed to connect to Wi-Fi AP.\n");
        CY_ASSERT(0);
    }

    /* Set event callback */
    if (cy_wcm_register_event_callback(wifi_event_callback) != CY_RSLT_SUCCESS) {
    printf("Failed to register network event callback\r\n");
        CY_ASSERT(0);
    }

    /* Create a queue */
    queue = xQueueCreate(QUEUE_SIZE, sizeof(server_ack_t));
    if (queue == NULL) {
        CY_ASSERT(false);
    }

    /* Secure Sockets initialization */
    result = cy_socket_init();
    if (result != CY_RSLT_SUCCESS) {
        printf("Secure Sockets initialization failed!\n");
        CY_ASSERT(0);
    }
    printf("Secure Sockets initialized\n");

    /* Create UDP Server */
    result = create_udp_server_socket();
    if (result != CY_RSLT_SUCCESS) {
        printf("UDP Server Socket creation failed. Error: %" PRIu32 "\n", result);
        CY_ASSERT(0);
    }

    server_ack_t ack;
    UBaseType_t ret;

    /* Crete NTP thread */
    ret = xTaskCreate(ntp_thread, "NTP task", NTP_SERVER_TASK_STACK_SIZE, NULL, NTP_SERVER_TASK_STACK_SIZE, &ntp_handle);
    if (ret != pdPASS)
    {
        CY_ASSERT(0);
    }

    /* Handle communication with the client */
    while (true) {
        xQueueReceive(queue, (void *const)&ack, portMAX_DELAY);

        result = cy_socket_sendto(server_handle, (void *const)&ack,
                            sizeof(server_ack_t), CY_SOCKET_FLAGS_NONE,
                            &client_addr, client_addr_len, &bytes_sent);

        if (result != CY_RSLT_SUCCESS) {
            printf("Failed to transmit to the client\r\n");
        }
    }
}

/**
 * @brief Create a wifi access point. The PSoC will host a Wifi access point, to
 * which a computer will have to connect in order to communicate with the device
 * wirelessly.
 *
 * @return cy_rslt_t
 */
cy_rslt_t create_wifi_ap(void) {
  cy_rslt_t result;
  cy_wcm_ap_config_t ap_conf;
  cy_wcm_ip_address_t ipv4_addr;

  cy_wcm_config_t wifi_config = {.interface = CY_WCM_INTERFACE_TYPE_AP_STA};

  /* Initialize Wi-Fi connection manager. */
  result = cy_wcm_init(&wifi_config);
  if (result != CY_RSLT_SUCCESS) {
    printf("Failed to setup WiFI AP\n");
  }

  memset(&ap_conf, 0, sizeof(cy_wcm_ap_config_t));
  memset(&ipv4_addr, 0, sizeof(cy_wcm_ip_address_t));

  ap_conf.channel = 1;
  memcpy(ap_conf.ap_credentials.SSID, SOFTAP_SSID, strlen(SOFTAP_SSID) + 1);
  memcpy(ap_conf.ap_credentials.password, SOFTAP_PASSWORD,
         strlen(SOFTAP_PASSWORD) + 1);

  ap_conf.ap_credentials.security = SOFTAP_SECURITY_TYPE;
  ap_conf.ip_settings.ip_address = ap_sta_mode_ip_settings.ip_address;
  ap_conf.ip_settings.netmask = ap_sta_mode_ip_settings.netmask;
  ap_conf.ip_settings.gateway = ap_sta_mode_ip_settings.gateway;

  result = cy_wcm_start_ap(&ap_conf);
  if (result != CY_RSLT_SUCCESS) {
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
cy_rslt_t create_udp_server_socket(void) {
    cy_rslt_t result;

    /* Variable used to set socket options. */
    cy_socket_opt_callback_t udp_recv_option = {
        .callback = udp_server_recv_handler,
        .arg = NULL
    };

    cy_socket_opt_callback_t ntp_client_option = {
        .callback = ntp_receive_handler,
        .arg = NULL
    };

    udp_server_addr.port = UDP_SERVER_PORT; // Your server port
    udp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;
    udp_server_addr.ip_address.ip.v4 = CY_SOCKET_INADDR_ANY;

    // NTP client
    ntp_server_addr.port = NTP_CLIENT_PORT; // NTP server port
    ntp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;
    ntp_server_addr.ip_address.ip.v4 = CY_SOCKET_INADDR_ANY;

    /* Create a UDP server socket. */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM,
                                CY_SOCKET_IPPROTO_UDP, &server_handle);
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }

    /* Register the callback function to handle messages received from UDP client.
    */
    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                    CY_SOCKET_SO_RECEIVE_CALLBACK, &udp_recv_option,
                                    sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }

    /* Bind the UDP socket created to Server IP address and port. */
    result = cy_socket_bind(server_handle, &udp_server_addr, sizeof(udp_server_addr));
    if (result == CY_RSLT_SUCCESS) {
        printf("Socket bound to port: %d\n", udp_server_addr.port);
    }

    /* Create the socket */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM,
                                CY_SOCKET_IPPROTO_UDP, &ntp_socket_handle);
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }

    result = cy_socket_setsockopt(ntp_socket_handle, CY_SOCKET_SOL_SOCKET,
                                    CY_SOCKET_SO_RECEIVE_CALLBACK, &ntp_client_option,
                                    sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS) {
        return result;
    }
    
    result = cy_socket_bind(ntp_socket_handle, &ntp_server_addr, sizeof(ntp_server_addr));
    if (result == CY_RSLT_SUCCESS) {
        printf("Socket bound to port: %d\n", ntp_server_addr.port);
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
    static bool task_resumed = false;

    /* Variable to store the number of bytes received. */
    uint32_t bytes_received = 0;
    client_req_t *req = NULL;
    server_ack_t reply;
    BaseType_t ret;

    /* Buffer to store data received from Client. */
    char message_buffer[MAX_UDP_RECV_BUFFER_SIZE] = {0};

    /* Receive incoming message from UDP server. */
    result = cy_socket_recvfrom(server_handle, message_buffer,
                                sizeof(client_req_t), CY_SOCKET_FLAGS_NONE,
                                &client_addr, &client_addr_len, &bytes_received);

    /* Wake up the NTP task */
    if ( !task_resumed )
    {
        vTaskResume(ntp_handle);
        task_resumed = true;
    }

    if (result != CY_RSLT_SUCCESS) {
        return result;
    }

    /* Parse the command */
    if (bytes_received != sizeof(client_req_t)) {
        return UDP_MESSAGE_SIZE_ERR;
    }

    req = (client_req_t *)(message_buffer);
    if (req->msg_length != (bytes_received - sizeof(uint32_t) - sizeof(uint16_t) -
                            sizeof(uint32_t))) {
        return UDP_MESSAGE_SIZE_ERR;
    }

    const uint32_t crc = crc32((char *)&req->payload, sizeof(req->payload));
    if (req->crc_32 != crc) {
        return UDP_CRC_SIZE_ERR;
    }

    /* Process the incoming network command and submit a reply to the client */
    ret = udp_reply_callback(req, sizeof(client_req_t));
    if (!ret) {
        reply.payload.command = 0;
        reply.payload.sequence_id = 0;
    } else {
        reply.payload.command = req->payload.command;
        reply.payload.sequence_id = req->sequence_id;
    }

    /* Reply to the client */
    ret = xQueueSend(queue, (const void *)&reply, portMAX_DELAY);
    if (!ret) {
        return UDP_REPLY_ERR;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
 * Function Name: ntp_send
 *******************************************************************************
 * Summary:
 *  Function to send NTP packet to NTP server
 *
 * Parameters:
 *  ntp_packet_t *ntp_packet : Pointer to NTP packet
 *
 * Return:
 *  bool : true if NTP packet is sent successfully, false otherwise
 *
 *******************************************************************************/
cy_rslt_t ntp_receive_handler(cy_socket_t socket_handle, void *arg)
{
    cy_rslt_t result;
    uint32_t bytes_received = 0;

    ntp_packet_t ntp_packet = {0};

    result = cy_socket_recvfrom(ntp_socket_handle, (void *)&ntp_packet,
                                sizeof(ntp_packet_t), CY_SOCKET_FLAGS_NONE,
                                &client_addr, &client_addr_len, &bytes_received);

    if (result != CY_RSLT_SUCCESS) {
        return result;
    }

    ntp_packet_t* req = &ntp_packet;

    // Convert received timestamps from network to host byte order
    const uint32_t T1      = lwip_ntohl(req->originate_timestamp[0]);
    const uint32_t T1_frac = lwip_ntohl(req->originate_timestamp[1]);

    const uint32_t T2      = lwip_ntohl(req->receive_timestamp[0]);
    const uint32_t T2_frac = lwip_ntohl(req->receive_timestamp[1]);

    const uint32_t T3      = lwip_ntohl(req->transmit_timestamp[0]);
    const uint32_t T3_frac = lwip_ntohl(req->transmit_timestamp[1]);

    // Get local receive time (T4)
    struct timeval time;
    xGetTimeTV(&time);

    const uint32_t T4      = (uint32_t)time.tv_sec;
    const uint32_t T4_frac = (uint32_t)(((uint64_t)time.tv_usec << 32) / 1000000);

    // Calculate time offset: ((T2 - T1) + (T3 - T4)) / 2
    int32_t offset      = ((int32_t)(T2 - T1) + (int32_t)(T3 - T4)) / 2;
    int32_t offset_frac = ((int32_t)(T2_frac - T1_frac) + (int32_t)(T3_frac - T4_frac)) / 2;

    if ( (offset < -1) || (offset > 1) ) {
        adjustTimer(offset, offset_frac, CLOCK_STEP);
    } else {
        adjustTimer(offset, offset_frac, CLOCK_SLEW);
    }

    // Store last valid NTP packet
    memcpy(&ntp, req, sizeof(ntp_packet_t));

    // Update LI/VN/Mode field — e.g. to version 4 server, no warning
    ntp.li_vn_mode = (0 << 6) | (4 << 3) | 3;  // LI=0, VN=4, Mode=3 (server)

    ntp_state = NTP_SLEEP;

    return CY_RSLT_SUCCESS;
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
void isr_button_press(void *callback_arg, cyhal_gpio_event_t event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Variable to hold the LED ON/OFF command to be sent to the UDP client. */
    uint32_t led_state_cmd;

    /* Set the command to be sent to UDP client. */
    if (led_state == CYBSP_LED_STATE_ON) {
        led_state_cmd = LED_OFF_CMD;
    } else {
        led_state_cmd = LED_ON_CMD;
    }

    /* Set the flag to send command to UDP client. */
    xTaskNotifyFromISR(server_task_handle, led_state_cmd,
                        eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Wifi event callback
 *
 * @param event Event triggered. See cy_wcm.c for event definition.
 * @param event_data
 */
void wifi_event_callback(cy_wcm_event_t event,
                         cy_wcm_event_data_t *event_data) {
    switch (event) {
        case CY_WCM_EVENT_STA_JOINED_SOFTAP:
            printf("Client connected to AP\n");
            cyhal_gpio_write(USER_LED, false);
            break;

        case CY_WCM_EVENT_STA_LEFT_SOFTAP:
            printf("Client disconnected from AP\n");
            cyhal_gpio_write(USER_LED, true);
            break;

        default:
            break;
    }
}


/**
 * @brief NTP thread
 * 
 * @param arg 
 */
static void ntp_thread( void* arg ) {

    ( void ) arg;
    vTaskSuspend(ntp_handle);

    while (1)
    {
        switch ( ntp_state )
        {
            case NTP_WAIT_FOR_ARP:{

                break;
            }

            case NTP_REQUEST: {
                struct timeval time;
                
                /* Get the timestamp */
                xGetTimeTV( &time );
                
                ntp.transmit_timestamp[0] = lwip_htonl(time.tv_sec + UNIX_OFFSET);
                ntp.transmit_timestamp[1] = lwip_htonl(MICROSECONDS_TO_FRACTION(time.tv_usec));
                ntp_do_send( &ntp );

                ntp_state = NTP_REPLY;
                break;
            }

            case NTP_REPLY: {
                uint8_t counter = 0;
                while ( counter < 15 ) {
                    vTaskDelay( 1000 );
                    counter++;
                }

                ntp_state = NTP_SLEEP;
                break;
            }
                
            case NTP_SLEEP:
                vTaskDelay( 5000 );

                ntp_state = NTP_REQUEST;
                break;
        }
    }
}


/*******************************************************************************
 * Function Name: ntp_send
 *******************************************************************************
 * Summary:
 *  Function to send NTP packet to NTP server
 *
 * Parameters:
 *  ntp_packet_t *ntp_packet : Pointer to NTP packet
 *
 * Return:
 *  bool : true if NTP packet is sent successfully, false otherwise
 *
 *******************************************************************************/
bool ntp_do_send(ntp_packet_t *ntp_packet) {
    if (ntp_packet == NULL) {
        return false;
    }

    cy_rslt_t result;
    uint32_t bytes_sent = 0;

    cy_socket_sockaddr_t ntp_server_addr;
    ntp_server_addr.port = NTP_PORT;
    ntp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;
    ntp_server_addr.ip_address.ip.v4 = client_addr.ip_address.ip.v4;

    /* Send the NTP packet */
    result = cy_socket_sendto(ntp_socket_handle, (void *const)ntp_packet,
                              sizeof(ntp_packet_t), CY_SOCKET_FLAGS_NONE,
                              &ntp_server_addr, sizeof(ntp_server_addr), &bytes_sent);

    if ( result != CY_RSLT_SUCCESS ) {
        return false;
    }

    return true;
}

/* [] END OF FILE */
