#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H


#define INITIALISER_IPV4_ADDRESS(addr_var, addr_val)  addr_var = { CY_WCM_IP_VER_V4, { .v4 = (uint32_t)(addr_val) } }
#define MAKE_IPV4_ADDRESS(a, b, c, d)                 ((((uint32_t) d) << 24) | (((uint32_t) c) << 16) | \
                                                       (((uint32_t) b) << 8) |((uint32_t) a))

/* The password length should meet the requirement of the configured security
 * type. e.g. Passworld length should be between 8-63 characters for
 * CY_WCM_SECURITY_WPA2_AES_PSK.
 */
#define SOFTAP_PASSWORD                              ""

/* SoftAP Credentials */
#define SOFTAP_SSID                                  "Carlos's RC Car"

#define SOFTAP_SECURITY_TYPE                         CY_WCM_SECURITY_OPEN
#define SOFTAP_IP_ADDRESS                            MAKE_IPV4_ADDRESS(192, 168, 1,  10)
#define SOFTAP_NETMASK                               MAKE_IPV4_ADDRESS(255, 255, 255, 0)
#define SOFTAP_GATEWAY                               MAKE_IPV4_ADDRESS(192, 168, 1,  100)

#endif