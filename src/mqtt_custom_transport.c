
/*
 * Custom MQTT Transport with Interface Binding
 *
 * This implementation provides the required custom transport functions
 * for Zephyr MQTT library with interface binding support.
 *
 * To use this:
 * 1. Enable CONFIG_MQTT_LIB_CUSTOM_TRANSPORT in your project configuration
 * 2. Set client->transport.type = MQTT_TRANSPORT_CUSTOM
 * 3. Store interface name in client->transport.custom_transport_data
 */

#include "mqtt_custom_transport.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mqtt_custom_transport, CONFIG_MQTT_LOG_LEVEL);

// NOTE - MUST BE DEFINED IN HEADER SO OTHER CODE CAN USE THIS STRUCT
// /* Structure to hold our custom transport data */
// struct custom_transport_data
// {
//   int  sock;
//   char interface_name[IFNAMSIZ];
// };

/**
 * Custom transport connect function with interface binding
 * This function is called by the MQTT library when transport.type = MQTT_TRANSPORT_CUSTOM
 */
int mqtt_client_custom_transport_connect(struct mqtt_client* client)
{
  if (client == NULL)
  {
    LOG_ERR("MQTT client is NULL");
    return -EINVAL;
  }

  const struct sockaddr* broker = client->broker;
  if (broker == NULL)
  {
    LOG_ERR("Broker address is NULL");
    return -EINVAL;
  }

  struct custom_transport_data* custom_data =
      (struct custom_transport_data*) client->transport.custom_transport_data;

  if (custom_data == NULL)
  {
    LOG_ERR("Custom transport data not initialized");
    return -EINVAL;
  }

  LOG_INF("Creating custom transport connection to broker with interface: %s",
          custom_data->interface_name);

  // Create socket
  int sock = zsock_socket(broker->sa_family, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0)
  {
    LOG_ERR("Failed to create socket: %d", errno);
    return -errno;
  }

  LOG_DBG("Created socket: %d", sock);

  // Bind socket to specified interface
  struct ifreq ifreq;
  memset(&ifreq, 0, sizeof(ifreq));
  strncpy(ifreq.ifr_name, custom_data->interface_name, sizeof(ifreq.ifr_name) - 1);

  int ret = zsock_setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifreq, sizeof(ifreq));
  if (ret < 0)
  {
    LOG_ERR("Failed to bind socket to interface %s: %d", custom_data->interface_name, errno);
    zsock_close(sock);
    return -errno;
  }

  LOG_DBG("Successfully bound socket %d to interface: %s", sock, custom_data->interface_name);

  // Connect to broker
  size_t peer_addr_size = sizeof(struct sockaddr_in6);
  if (broker->sa_family == AF_INET)
  {
    peer_addr_size = sizeof(struct sockaddr_in);
  }

  ret = zsock_connect(sock, broker, peer_addr_size);
  if (ret < 0)
  {
    LOG_ERR("Failed to connect to broker: %d", errno);
    zsock_close(sock);
    return -errno;
  }

  // Store the socket in our custom data
  custom_data->sock = sock;

  LOG_INF("Successfully connected to MQTT broker via interface %s (socket %d)",
          custom_data->interface_name,
          sock);

  return 0;
}

/**
 * Custom transport write function
 */
int mqtt_client_custom_transport_write(struct mqtt_client* client,
                                       const uint8_t*      data,
                                       uint32_t            datalen)
{
  if (client == NULL || data == NULL)
  {
    return -EINVAL;
  }

  struct custom_transport_data* custom_data =
      (struct custom_transport_data*) client->transport.custom_transport_data;

  if (custom_data == NULL || custom_data->sock < 0)
  {
    LOG_ERR("Invalid custom transport data or socket");
    return -EINVAL;
  }

  uint32_t offset = 0U;
  int      ret;

  while (offset < datalen)
  {
    ret = zsock_send(custom_data->sock, data + offset, datalen - offset, 0);
    if (ret < 0)
    {
      LOG_ERR("Socket send failed: %d", errno);
      return -errno;
    }
    offset += ret;
  }

  LOG_DBG("Sent %u bytes via custom transport", datalen);
  return 0;
}

/**
 * Custom transport write message function
 */
int mqtt_client_custom_transport_write_msg(struct mqtt_client* client, const struct msghdr* message)
{
  if (client == NULL || message == NULL)
  {
    return -EINVAL;
  }

  struct custom_transport_data* custom_data =
      (struct custom_transport_data*) client->transport.custom_transport_data;

  if (custom_data == NULL || custom_data->sock < 0)
  {
    LOG_ERR("Invalid custom transport data or socket");
    return -EINVAL;
  }

  int    ret, i;
  size_t offset    = 0;
  size_t total_len = 0;

  // Calculate total message length
  for (i = 0; i < message->msg_iovlen; i++)
  {
    total_len += message->msg_iov[i].iov_len;
  }

  while (offset < total_len)
  {
    ret = zsock_sendmsg(custom_data->sock, message, 0);
    if (ret < 0)
    {
      LOG_ERR("Socket sendmsg failed: %d", errno);
      return -errno;
    }
    offset += ret;
    if (offset >= total_len)
    {
      break;
    }

    /* Update msghdr for the next iteration. */
    for (i = 0; i < message->msg_iovlen; i++)
    {
      if (ret < message->msg_iov[i].iov_len)
      {
        message->msg_iov[i].iov_len -= ret;
        message->msg_iov[i].iov_base = (uint8_t*) message->msg_iov[i].iov_base + ret;
        break;
      }
      ret -= message->msg_iov[i].iov_len;
      message->msg_iov[i].iov_len = 0;
    }
  }

  LOG_DBG("Sent message (%zu bytes) via custom transport", total_len);
  return 0;
}

/**
 * Custom transport read function
 */
int mqtt_client_custom_transport_read(struct mqtt_client* client,
                                      uint8_t*            data,
                                      uint32_t            buflen,
                                      bool                shall_block)
{
  if (client == NULL || data == NULL)
  {
    return -EINVAL;
  }

  struct custom_transport_data* custom_data =
      (struct custom_transport_data*) client->transport.custom_transport_data;

  if (custom_data == NULL || custom_data->sock < 0)
  {
    LOG_ERR("Invalid custom transport data or socket");
    return -EINVAL;
  }

  int flags = 0;
  if (!shall_block)
  {
    flags |= ZSOCK_MSG_DONTWAIT;
  }

  int ret = zsock_recv(custom_data->sock, data, buflen, flags);
  if (ret < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      LOG_ERR("Socket recv failed: %d", errno);
    }
    return -errno;
  }

  if (ret > 0)
  {
    LOG_DBG("Received %d bytes via custom transport", ret);
  }

  return ret;
}

/**
 * Custom transport disconnect function
 */
int mqtt_client_custom_transport_disconnect(struct mqtt_client* client)
{
  if (client == NULL)
  {
    return -EINVAL;
  }

  struct custom_transport_data* custom_data =
      (struct custom_transport_data*) client->transport.custom_transport_data;

  if (custom_data == NULL)
  {
    LOG_WRN("Custom transport data is NULL during disconnect");
    return 0; // Not an error, might already be cleaned up
  }

  if (custom_data->sock >= 0)
  {
    LOG_INF("Closing custom transport socket %d", custom_data->sock);

    int ret = zsock_close(custom_data->sock);
    if (ret < 0)
    {
      LOG_ERR("Failed to close socket: %d", errno);
      return -errno;
    }

    custom_data->sock = -1;
  }

  return 0;
}

/* Helper functions for easier usage */

/**
 * Initialize MQTT client with custom interface-bound transport
 */
int mqtt_client_init_with_interface(struct mqtt_client*           client,
                                    const char*                   interface_name,
                                    struct custom_transport_data* transport_data)
{
  if (client == NULL || interface_name == NULL || transport_data == NULL)
  {
    LOG_ERR("Invalid parameters for interface initialization");
    return -EINVAL;
  }

  // Initialize the MQTT client normally
  mqtt_client_init(client);

  // Set up our custom transport data
  memset(transport_data, 0, sizeof(struct custom_transport_data));
  strncpy(
      transport_data->interface_name, interface_name, sizeof(transport_data->interface_name) - 1);
  transport_data->sock = -1;

  // Configure client to use custom transport
  client->transport.type                  = MQTT_TRANSPORT_CUSTOM;
  client->transport.custom_transport_data = transport_data;

  LOG_INF("MQTT client initialized with interface binding to: %s", interface_name);

  return 0;
}

/**
 * Connect MQTT client using custom interface-bound transport
 *
 * This function can be used as a drop-in replacement for mqtt_connect()
 * when you want interface binding.
 */
int mqtt_connect_with_interface(struct mqtt_client* client, const char* interface_name)
{
  if (client == NULL || interface_name == NULL)
  {
    LOG_ERR("Invalid parameters for interface connection");
    return -EINVAL;
  }

  // Allocate transport data (you might want to make this static or part of client)
  static struct custom_transport_data transport_data;

  // Initialize custom transport
  int ret = mqtt_client_init_with_interface(client, interface_name, &transport_data);
  if (ret < 0)
  {
    LOG_ERR("Failed to initialize client with interface: %d", ret);
    return ret;
  }

  // Now call the normal MQTT connect - it will use our custom transport
  ret = mqtt_connect(client);
  if (ret < 0)
  {
    LOG_ERR("MQTT connect failed: %d", ret);
    return ret;
  }

  LOG_INF("MQTT connected successfully with interface binding to %s", interface_name);
  return 0;
}
