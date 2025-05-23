/*
 * MQTT API Implementation
 * Provides interface for MQTT operations with interface binding support
 */

#include "mqtt_api.h"

#include "mqtt_custom_transport.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(mqtt_api, LOG_LEVEL_DBG);

/* MQTT Configuration */
#define MQTT_CLIENT_ID_PREFIX "nrf9151_demo"
#define MQTT_KEEPALIVE_SEC 60
#define MQTT_CLEAN_SESSION 1
#define MQTT_RX_BUFFER_SIZE 512
#define MQTT_TX_BUFFER_SIZE 512
#define MQTT_PAYLOAD_BUFFER_SIZE 256

/* MQTT Client State */
static struct mqtt_client           mqtt_client_ctx;
static struct custom_transport_data mqtt_transport_data;
static bool                         mqtt_connected              = false;
static char                         current_interface[IFNAMSIZ] = {0};

/* MQTT Buffers */
static uint8_t mqtt_rx_buffer[MQTT_RX_BUFFER_SIZE];
static uint8_t mqtt_tx_buffer[MQTT_TX_BUFFER_SIZE];
static uint8_t mqtt_payload_buffer[MQTT_PAYLOAD_BUFFER_SIZE];

/* Semaphores */
static K_SEM_DEFINE(mqtt_conn_sem, 0, 1);

/**
 * @brief Generate unique client ID based on device info
 */
static void generate_client_id(char* client_id, size_t max_len)
{
  /* Try to use MAC address from default interface */
  struct net_if* iface = net_if_get_default();

  if (iface && iface->if_dev && iface->if_dev->link_addr.addr)
  {
    const uint8_t* mac = iface->if_dev->link_addr.addr;
    snprintf(client_id, max_len, "%s_%02x%02x%02x", MQTT_CLIENT_ID_PREFIX, mac[3], mac[4], mac[5]);
  }
  else
  {
    /* Fallback to random ID */
    snprintf(client_id, max_len, "%s_%08x", MQTT_CLIENT_ID_PREFIX, sys_rand32_get());
  }

  LOG_INF("Generated MQTT client ID: %s", client_id);
}

/**
 * @brief MQTT event handler
 */
static void mqtt_evt_handler(struct mqtt_client* client, const struct mqtt_evt* evt)
{
  LOG_INF("MQTT event received: type=%d, result=%d", evt->type, evt->result);

  switch (evt->type)
  {
    case MQTT_EVT_CONNACK:
      if (evt->result == 0)
      {
        LOG_INF("MQTT client connected successfully!");
        mqtt_connected = true;
        k_sem_give(&mqtt_conn_sem);
      }
      else
      {
        LOG_ERR("MQTT connection rejected by broker: %d", evt->result);
        mqtt_connected = false;
        k_sem_give(&mqtt_conn_sem); // Still signal to unblock waiting thread
      }
      break;

    case MQTT_EVT_DISCONNECT:
      LOG_WRN("MQTT client disconnected: %d", evt->result);
      mqtt_connected = false;
      k_sem_take(&mqtt_conn_sem, K_NO_WAIT); // Reset semaphore
      break;

    case MQTT_EVT_PUBLISH:
      LOG_INF("MQTT publish received on topic");
      /* Note: Payload needs to be read separately with mqtt_read_publish_payload */
      break;

    case MQTT_EVT_PUBACK:
      LOG_DBG("MQTT PUBACK received for message ID: %d", evt->param.puback.message_id);
      break;

    case MQTT_EVT_SUBACK:
      LOG_INF("MQTT SUBACK received for message ID: %d", evt->param.suback.message_id);
      break;

    case MQTT_EVT_PINGRESP:
      LOG_DBG("MQTT PINGRESP received");
      break;

    default:
      LOG_DBG("MQTT event type: %d", evt->type);
      break;
  }
}

/**
 * @brief Setup broker address from hostname or IP
 */
static int
setup_broker_address(struct sockaddr_in* broker_addr, const char* hostname, uint16_t port)
{
  /* First try to parse as IP address directly */
  memset(broker_addr, 0, sizeof(*broker_addr));
  broker_addr->sin_family = AF_INET;
  broker_addr->sin_port   = htons(port);

  int ret = inet_pton(AF_INET, hostname, &broker_addr->sin_addr);
  if (ret == 1)
  {
    /* Successfully parsed as IP address */
    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &broker_addr->sin_addr, addr_str, sizeof(addr_str));
    LOG_INF("Using IP address %s:%d for broker", addr_str, port);
    return 0;
  }

  /* Not an IP address, try DNS resolution */
  struct addrinfo hints, *result;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  ret = getaddrinfo(hostname, NULL, &hints, &result);
  if (ret != 0)
  {
    LOG_ERR("getaddrinfo failed for %s: %d", hostname, ret);

    /* Fallback to hardcoded IP for broker.hivemq.com */
    if (strcmp(hostname, "broker.hivemq.com") == 0)
    {
      LOG_WRN("DNS failed, using fallback IP 18.185.216.207 for broker.hivemq.com");
      ret = inet_pton(AF_INET, "18.185.216.207", &broker_addr->sin_addr);
      if (ret == 1)
      {
        return 0;
      }
    }

    return -EHOSTUNREACH;
  }

  if (result == NULL)
  {
    LOG_ERR("No addresses found for %s", hostname);
    return -ENOENT;
  }

  /* Copy the resolved address */
  memcpy(broker_addr, result->ai_addr, sizeof(struct sockaddr_in));
  broker_addr->sin_port = htons(port);

  char addr_str[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &broker_addr->sin_addr, addr_str, sizeof(addr_str));
  LOG_INF("Resolved %s to %s:%d", hostname, addr_str, port);

  freeaddrinfo(result);
  return 0;
}

int mqtt_api_init(void)
{
  LOG_INF("Initializing MQTT API");

  /* Reset connection state */
  mqtt_connected = false;
  memset(current_interface, 0, sizeof(current_interface));

  LOG_INF("MQTT API initialized successfully");
  return 0;
}

int mqtt_api_connect(const char* interface_name)
{
  int                       ret;
  static struct sockaddr_in broker_addr;
  static char               client_id[64];

  if (interface_name == NULL)
  {
    LOG_ERR("Invalid interface name");
    return -EINVAL;
  }

  if (mqtt_connected)
  {
    LOG_WRN("MQTT already connected, disconnecting first");
    mqtt_api_disconnect();
  }

  LOG_INF("Connecting to MQTT broker via interface: %s", interface_name);

  /* Setup broker address */
  ret = setup_broker_address(&broker_addr, MQTT_BROKER_HOSTNAME, MQTT_BROKER_PORT);
  if (ret)
  {
    LOG_ERR("Failed to setup broker address: %d", ret);
    return ret;
  }

  /* Initialize MQTT client with custom transport */
  ret = mqtt_client_init_with_interface(&mqtt_client_ctx, interface_name, &mqtt_transport_data);
  if (ret)
  {
    LOG_ERR("Failed to initialize MQTT client: %d", ret);
    return ret;
  }

  /* Generate client ID */
  generate_client_id(client_id, sizeof(client_id));

  /* Configure MQTT client parameters */
  mqtt_client_ctx.broker           = (struct sockaddr*) &broker_addr;
  mqtt_client_ctx.evt_cb           = mqtt_evt_handler;
  mqtt_client_ctx.client_id.utf8   = (uint8_t*) client_id;
  mqtt_client_ctx.client_id.size   = strlen(client_id);
  mqtt_client_ctx.protocol_version = MQTT_VERSION_3_1_1;
  mqtt_client_ctx.clean_session    = MQTT_CLEAN_SESSION;
  mqtt_client_ctx.keepalive        = MQTT_KEEPALIVE_SEC;

  /* Optional: Set user credentials (leave NULL for anonymous) */
  mqtt_client_ctx.user_name = NULL;
  mqtt_client_ctx.password  = NULL;

  /* Set up buffers */
  mqtt_client_ctx.rx_buf      = mqtt_rx_buffer;
  mqtt_client_ctx.rx_buf_size = sizeof(mqtt_rx_buffer);
  mqtt_client_ctx.tx_buf      = mqtt_tx_buffer;
  mqtt_client_ctx.tx_buf_size = sizeof(mqtt_tx_buffer);

  /* Connect to broker */
  LOG_INF("Sending MQTT CONNECT packet to broker...");
  ret = mqtt_connect(&mqtt_client_ctx);
  if (ret)
  {
    LOG_ERR("Failed to initiate MQTT connection: %d", ret);
    return ret;
  }

  /* Wait for connection to complete with longer timeout */
  LOG_INF("Waiting for MQTT CONNACK...");

  /* Process MQTT input while waiting for CONNACK */
  for (int i = 0; i < 60; i++)
  { // Wait up to 60 seconds
    /* Process any incoming MQTT messages */
    mqtt_input(&mqtt_client_ctx);

    /* Check if we got the CONNACK */
    if (k_sem_take(&mqtt_conn_sem, K_MSEC(100)) == 0)
    {
      /* Got CONNACK, check if connection was successful */
      if (mqtt_connected)
      {
        break;
      }
      else
      {
        LOG_ERR("MQTT connection was rejected by broker");
        return -ECONNREFUSED;
      }
    }

    /* Small delay between checks */
    k_sleep(K_MSEC(900)); // Total 1 second per loop
  }

  if (!mqtt_connected)
  {
    LOG_ERR("MQTT connection timeout - no CONNACK received");
    mqtt_api_disconnect(); // Clean up
    return -ETIMEDOUT;
  }

  /* Store current interface */
  strncpy(current_interface, interface_name, sizeof(current_interface) - 1);

  LOG_INF("MQTT connected successfully via interface: %s", interface_name);
  return 0;
}

int mqtt_api_disconnect(void)
{
  int ret;

  if (!mqtt_connected)
  {
    LOG_WRN("MQTT not connected");
    return 0;
  }

  LOG_INF("Disconnecting from MQTT broker");

  ret = mqtt_disconnect(&mqtt_client_ctx, NULL);
  if (ret)
  {
    LOG_ERR("Failed to disconnect from MQTT broker: %d", ret);
  }

  mqtt_connected = false;
  memset(current_interface, 0, sizeof(current_interface));

  LOG_INF("MQTT disconnected");
  return ret;
}

bool mqtt_api_is_connected(void)
{
  return mqtt_connected;
}

int mqtt_api_publish(const char*    topic,
                     const uint8_t* payload,
                     size_t         payload_len,
                     enum mqtt_qos  qos)
{
  int                       ret;
  struct mqtt_publish_param param;
  static uint16_t           message_id = 1;

  if (!mqtt_connected)
  {
    LOG_ERR("MQTT not connected");
    return -ENOTCONN;
  }

  if (topic == NULL || payload == NULL)
  {
    LOG_ERR("Invalid parameters");
    return -EINVAL;
  }

  /* Set up publish parameters */
  memset(&param, 0, sizeof(param));
  param.message.topic.topic.utf8 = (uint8_t*) topic;
  param.message.topic.topic.size = strlen(topic);
  param.message.topic.qos        = qos;
  param.message.payload.data     = (uint8_t*) payload;
  param.message.payload.len      = payload_len;
  param.message_id               = message_id++;
  param.dup_flag                 = 0;
  param.retain_flag              = 0;

  LOG_INF("Publishing message to topic '%s' (QoS %d, %zu bytes)", topic, qos, payload_len);
  LOG_DBG("Payload: %.*s", (int) payload_len, payload);

  ret = mqtt_publish(&mqtt_client_ctx, &param);
  if (ret)
  {
    LOG_ERR("Failed to publish MQTT message: %d", ret);
    return ret;
  }

  LOG_INF("Message published successfully (ID: %d)", param.message_id);
  return 0;
}

int mqtt_api_publish_status(uint32_t message_id)
{
  char payload[128];
  int  len;

  if (!mqtt_connected)
  {
    LOG_ERR("MQTT not connected");
    return -ENOTCONN;
  }

  /* Create JSON status message */
  len = snprintf(payload,
                 sizeof(payload),
                 "{"
                 "\"message_id\":%u,"
                 "\"timestamp\":%lld,"
                 "\"device\":\"nRF9151\","
                 "\"interface\":\"%s\","
                 "\"uptime\":%lld,"
                 "}",
                 message_id,
                 k_uptime_get(),
                 current_interface,
                 k_uptime_get() / 1000);

  if (len >= sizeof(payload))
  {
    LOG_WRN("Status message truncated");
    len = sizeof(payload) - 1;
  }

  char topic[32];
  snprintf(topic, sizeof(topic), "%s/status", MQTT_TOPIC_PREFIX);

  return mqtt_api_publish(topic, (uint8_t*) payload, len, MQTT_QOS_1_AT_LEAST_ONCE);
}

int mqtt_api_process_input(void)
{
  int ret;

  if (!mqtt_connected)
  {
    return -ENOTCONN;
  }

  ret = mqtt_input(&mqtt_client_ctx);
  if (ret && ret != -EAGAIN)
  {
    LOG_ERR("MQTT input error: %d", ret);
    mqtt_connected = false;
  }

  return ret;
}

int mqtt_api_handle_keepalive(void)
{
  int ret;

  if (!mqtt_connected)
  {
    return -ENOTCONN;
  }

  ret = mqtt_live(&mqtt_client_ctx);
  if (ret && ret != -EAGAIN)
  {
    LOG_ERR("MQTT keepalive error: %d", ret);
    mqtt_connected = false;
  }

  return ret;
}

const char* mqtt_api_get_interface(void)
{
  if (!mqtt_connected || strlen(current_interface) == 0)
  {
    return NULL;
  }
  return current_interface;
}