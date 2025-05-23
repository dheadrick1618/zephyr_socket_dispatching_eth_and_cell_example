/*
 * MQTT API Header
 * Provides interface for MQTT operations with interface binding support
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/net/mqtt.h>

/* MQTT Configuration */
#define MQTT_BROKER_HOSTNAME "broker.hivemq.com"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC_PREFIX "nrf9151/demo"
#define MQTT_PUBLISH_INTERVAL_SEC 30

/* MQTT API Functions */

/**
 * @brief Initialize MQTT subsystem
 *
 * @return 0 on success, negative error code on failure
 */
int mqtt_api_init(void);

/**
 * @brief Connect to MQTT broker using specified interface
 *
 * @param interface_name Network interface name (e.g., "eth0", "net0")
 * @return 0 on success, negative error code on failure
 */
int mqtt_api_connect(const char* interface_name);

/**
 * @brief Disconnect from MQTT broker
 *
 * @return 0 on success, negative error code on failure
 */
int mqtt_api_disconnect(void);

/**
 * @brief Check if MQTT client is connected
 *
 * @return true if connected, false otherwise
 */
bool mqtt_api_is_connected(void);

/**
 * @brief Publish a message to a topic
 *
 * @param topic Topic to publish to
 * @param payload Message payload
 * @param payload_len Length of payload
 * @param qos Quality of Service level
 * @return 0 on success, negative error code on failure
 */
int mqtt_api_publish(const char*    topic,
                     const uint8_t* payload,
                     size_t         payload_len,
                     enum mqtt_qos  qos);

/**
 * @brief Publish a JSON formatted device status message
 *
 * @param message_id Unique message identifier
 * @return 0 on success, negative error code on failure
 */
int mqtt_api_publish_status(uint32_t message_id);

/**
 * @brief Process MQTT input (should be called regularly)
 *
 * @return 0 on success, negative error code on failure
 */
int mqtt_api_process_input(void);

/**
 * @brief Handle MQTT keepalive (should be called regularly)
 *
 * @return 0 on success, negative error code on failure
 */
int mqtt_api_handle_keepalive(void);

/**
 * @brief Get the current interface name being used
 *
 * @return Interface name string, or NULL if not connected
 */
const char* mqtt_api_get_interface(void);