
#pragma once

#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>

/* Structure to hold our custom transport data */
struct custom_transport_data
{
  int  sock;
  char interface_name[IFNAMSIZ];
};

int mqtt_client_custom_transport_connect(struct mqtt_client* client);

int mqtt_client_custom_transport_write(struct mqtt_client* client,
                                       const uint8_t*      data,
                                       uint32_t            datalen);

int mqtt_client_custom_transport_write_msg(struct mqtt_client*  client,
                                           const struct msghdr* message);

int mqtt_client_custom_transport_read(struct mqtt_client* client,
                                      uint8_t*            data,
                                      uint32_t            buflen,
                                      bool                shall_block);

int mqtt_client_custom_transport_disconnect(struct mqtt_client* client);

int mqtt_client_init_with_interface(struct mqtt_client*           client,
                                    const char*                   interface_name,
                                    struct custom_transport_data* transport_data);

int mqtt_connect_with_interface(struct mqtt_client* client, const char* interface_name);