/*
 * Socket Dispatcher Demo - Testing Multiple Network Interfaces with MQTT
 */

#include "mqtt_api.h"

#include <errno.h>
#include <fcntl.h> /* For O_NONBLOCK */
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <stdio.h> // For snprintf
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>

LOG_MODULE_REGISTER(socket_dispatcher_demo, LOG_LEVEL_INF);

/* Configuration */
#define UDP_SERVER_PORT 4242
#define UDP_SERVER_ADDR "8.8.8.8" /* Google's DNS server - easily accessible and accepts UDP */

#define TCP_SERVER_PORT 4242
#define TCP_SERVER_ADDR "45.79.112.203"

#define DNS_TEST_HOSTNAME "nordicsemi.com"

#define SOCKET_TIMEOUT_SEC 10
#define MODEM_CONN_TIMEOUT_SEC 60

/* LTE connection semaphore */
K_SEM_DEFINE(lte_connected, 0, 1);

/* Network interfaces */
#define IFACE_CELLULAR "net0"
#define IFACE_ETHERNET "eth0"

static struct net_mgmt_event_callback net_mgmt_cb;

/* LTE event handler */
static void lte_handler(const struct lte_lc_evt* const evt)
{
  if (evt->type == LTE_LC_EVT_NW_REG_STATUS)
  {
    if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
        (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING))
    {
      LOG_INF("Network registered");
      k_sem_give(&lte_connected);
    }
  }
}

/* Test a socket bound to a specific interface */
static int test_socket_binding(const char* iface_name)
{
  int          sock;
  int          ret;
  struct ifreq ifreq;

  LOG_INF("Testing socket creation on interface '%s'", iface_name);

  /* Create a socket */
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0)
  {
    LOG_ERR("Failed to create socket: %d", errno);
    return -errno;
  }

  /* Bind socket to interface */
  memset(&ifreq, 0, sizeof(ifreq));
  strncpy(ifreq.ifr_name, iface_name, sizeof(ifreq.ifr_name) - 1);

  ret = setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifreq, sizeof(ifreq));
  if (ret < 0)
  {
    LOG_ERR("Failed to bind socket to interface '%s': %d", iface_name, errno);
    close(sock);
    return -errno;
  }

  LOG_INF("Successfully created and bound socket to interface '%s'", iface_name);

  /* Close the socket */
  close(sock);
  LOG_INF("Socket closed");

  return 0;
}

/* Test UDP communication through specified interface */
static int test_udp(const char* iface_name)
{
  int                sock, ret;
  struct ifreq       ifreq;
  struct sockaddr_in remote_addr;
  char               send_buf[64]  = {0};
  char               recv_buf[128] = {0};

  snprintf(send_buf, sizeof(send_buf), "Hello from nRF9151 via %s!", iface_name);

  LOG_INF("Testing UDP through %s", iface_name);

  /* Create UDP socket */
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0)
  {
    LOG_ERR("Failed to create UDP socket: %d", errno);
    return -errno;
  }

  /* Bind to specified interface */
  memset(&ifreq, 0, sizeof(ifreq));
  strncpy(ifreq.ifr_name, iface_name, sizeof(ifreq.ifr_name) - 1);

  ret = setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifreq, sizeof(ifreq));
  if (ret < 0)
  {
    LOG_ERR("Failed to bind socket to %s interface: %d", iface_name, errno);
    close(sock);
    return -errno;
  }

  /* Try to set socket timeout - but handle if not supported */
  struct timeval timeout           = {.tv_sec = SOCKET_TIMEOUT_SEC, .tv_usec = 0};
  bool           timeout_supported = true;

  ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  if (ret < 0)
  {
    if (errno == ENOPROTOOPT || errno == ENOTSUP)
    {
      LOG_WRN("Socket timeout not supported on %s, using poll() instead", iface_name);
      timeout_supported = false;
    }
    else
    {
      LOG_WRN("Failed to set receive timeout: %d", errno);
    }
  }

  /* Set up remote address */
  memset(&remote_addr, 0, sizeof(remote_addr));
  remote_addr.sin_family = AF_INET;
  remote_addr.sin_port   = htons(UDP_SERVER_PORT);

  ret = inet_pton(AF_INET, UDP_SERVER_ADDR, &remote_addr.sin_addr);
  if (ret != 1)
  {
    LOG_ERR("Invalid address: %s", UDP_SERVER_ADDR);
    close(sock);
    return -EINVAL;
  }

  /* Send data */
  LOG_INF("Sending UDP packet to %s:%d via %s", UDP_SERVER_ADDR, UDP_SERVER_PORT, iface_name);

  ret = sendto(
      sock, send_buf, strlen(send_buf), 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));

  if (ret < 0)
  {
    LOG_ERR("Failed to send data: %d", errno);
    close(sock);
    return -errno;
  }

  LOG_INF("Sent %d bytes", ret);

  /* Try to receive a response */
  LOG_INF("Waiting for response...");

  if (timeout_supported)
  {
    /* Use regular recv with timeout set via setsockopt */
    ret = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);

    if (ret < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        LOG_INF("Timed out waiting for response (this is normal for DNS server)");
      }
      else
      {
        LOG_ERR("Failed to receive: %d", errno);
      }
    }
    else if (ret > 0)
    {
      recv_buf[ret] = '\0';
      LOG_INF("Received %d bytes: %s", ret, recv_buf);
    }
  }
  else
  {
    /* Use poll with manual timeout as a fallback */
    struct pollfd fds = {
        .fd     = sock,
        .events = POLLIN,
    };

    ret = poll(&fds, 1, SOCKET_TIMEOUT_SEC * 1000);

    if (ret < 0)
    {
      LOG_ERR("Poll error: %d", errno);
    }
    else if (ret == 0)
    {
      LOG_INF("Timed out waiting for response (this is normal for DNS server)");
    }
    else
    {
      ret = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);
      if (ret < 0)
      {
        LOG_ERR("Failed to receive data: %d", errno);
      }
      else if (ret > 0)
      {
        recv_buf[ret] = '\0';
        LOG_INF("Received %d bytes: %s", ret, recv_buf);
      }
    }
  }

  /* Close socket */
  close(sock);
  LOG_INF("UDP test on %s complete", iface_name);

  return 0;
}

/* Test DNS resolution through specified interface */
static int test_dns_query(const char* iface_name)
{
  int             sock, ret;
  struct ifreq    ifreq;
  struct addrinfo hints, *res;
  char            addr_str[INET6_ADDRSTRLEN];

  LOG_INF("Testing DNS resolution through %s", iface_name);

  /* Create socket (needed for DNS query) */
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0)
  {
    LOG_ERR("Failed to create socket: %d", errno);
    return -errno;
  }

  /* Bind to specified interface */
  memset(&ifreq, 0, sizeof(ifreq));
  strncpy(ifreq.ifr_name, iface_name, sizeof(ifreq.ifr_name) - 1);

  ret = setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifreq, sizeof(ifreq));
  if (ret < 0)
  {
    LOG_ERR("Failed to bind socket to %s interface: %d", iface_name, errno);
    close(sock);
    return -errno;
  }

  /* Perform DNS lookup */
  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  LOG_INF("Resolving %s via %s...", DNS_TEST_HOSTNAME, iface_name);
  ret = getaddrinfo(DNS_TEST_HOSTNAME, NULL, &hints, &res);

  if (ret != 0)
  {
    LOG_ERR("getaddrinfo failed: %d", ret);
    close(sock);
    return -EIO;
  }

  /* Print resolved address */
  struct sockaddr_in* addr = (struct sockaddr_in*) res->ai_addr;
  inet_ntop(AF_INET, &addr->sin_addr, addr_str, sizeof(addr_str));

  LOG_INF("Resolved %s to %s via %s", DNS_TEST_HOSTNAME, addr_str, iface_name);

  /* Free addrinfo and close socket */
  freeaddrinfo(res);
  close(sock);

  LOG_INF("DNS test on %s complete", iface_name);
  return 0;
}

/* Test TCP communication through specified interface */
static int test_tcp(const char* iface_name)
{
  int                sock, ret;
  struct ifreq       ifreq;
  struct sockaddr_in remote_addr;
  char               send_buf[64]      = {0};
  char               recv_buf[128]     = {0};
  bool               timeout_supported = true;

  snprintf(send_buf, sizeof(send_buf), "Hello from nRF9151 via %s\n", iface_name);

  LOG_INF("Testing TCP through %s", iface_name);

  // Create socket of type TCP
  sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0)
  {
    LOG_ERR("Failed to create TCP socket: %d", errno);
    return -errno;
  }

  // Bind socket to specified interface
  memset(&ifreq, 0, sizeof(ifreq));
  strncpy(ifreq.ifr_name, iface_name, sizeof(ifreq.ifr_name) - 1);

  ret = setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifreq, sizeof(ifreq));
  if (ret < 0)
  {
    LOG_ERR("Failed to bind socket to %s interface: %d", iface_name, errno);
    close(sock);
    return -errno;
  }

  // Try to set socket timeout
  struct timeval timeout = {.tv_sec = SOCKET_TIMEOUT_SEC, .tv_usec = 0};

  ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  if (ret < 0)
  {
    if (errno == ENOPROTOOPT || errno == ENOTSUP)
    {
      LOG_WRN("Socket timeout not supported on %s, using poll() instead", iface_name);
      timeout_supported = false;
    }
    else
    {
      LOG_WRN("Failed to set timeout: %d", errno);
    }
  }

  // Try to set connect timeout (non-blocking connect with poll)
  // This is needed regardless of timeout support since connect() can block
  if (!timeout_supported)
  {
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0)
    {
      LOG_ERR("Failed to get socket flags: %d", errno);
      close(sock);
      return -errno;
    }

    ret = fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0)
    {
      LOG_ERR("Failed to set socket non-blocking: %d", errno);
      close(sock);
      return -errno;
    }
  }

  // Setup remote address
  memset(&remote_addr, 0, sizeof(remote_addr));
  remote_addr.sin_family = AF_INET;
  remote_addr.sin_port   = htons(TCP_SERVER_PORT);

  ret = inet_pton(AF_INET, TCP_SERVER_ADDR, &remote_addr.sin_addr);
  if (ret != 1)
  {
    LOG_ERR("Invalid address: %s", TCP_SERVER_ADDR);
    close(sock);
    return -EINVAL;
  }

  // Connect to server
  LOG_INF("Connecting to TCP server %s:%d via %s...", TCP_SERVER_ADDR, TCP_SERVER_PORT, iface_name);

  ret = connect(sock, (struct sockaddr*) &remote_addr, sizeof(remote_addr));

  if (ret < 0)
  {
    if (!timeout_supported && (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK))
    {
      // This is expected for non-blocking connect - use poll to wait
      struct pollfd fds = {
          .fd     = sock,
          .events = POLLOUT,
      };

      ret = poll(&fds, 1, SOCKET_TIMEOUT_SEC * 1000);

      if (ret < 0)
      {
        LOG_ERR("Poll error during connect: %d", errno);
        close(sock);
        return -errno;
      }
      else if (ret == 0)
      {
        LOG_ERR("Connection timeout");
        close(sock);
        return -ETIMEDOUT;
      }
      else
      {
        // Check if connection was successful
        int       err;
        socklen_t err_len = sizeof(err);
        ret               = getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &err_len);

        if (ret < 0 || err != 0)
        {
          LOG_ERR("Connection failed: %d", err != 0 ? err : errno);
          close(sock);
          return -EIO;
        }

        // Set back to blocking mode for data transfer
        int flags = fcntl(sock, F_GETFL, 0);
        if (flags >= 0)
        {
          fcntl(sock, F_SETFL, flags & ~O_NONBLOCK);
        }
      }
    }
    else
    {
      LOG_ERR("Failed to connect to TCP server: %d", errno);
      close(sock);
      return -errno;
    }
  }

  LOG_INF("Connected to TCP server");

  // Send data
  ret = send(sock, send_buf, strlen(send_buf), 0);
  if (ret < 0)
  {
    LOG_ERR("Failed to send: %d", errno);
    close(sock);
    return -errno;
  }
  else
  {
    LOG_INF("Sent message to TCP server. Awaiting response...");
  }

  // Wait to receive response
  if (timeout_supported)
  {
    // Use regular recv with timeout set via setsockopt
    ret = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);

    if (ret < 0)
    {
      LOG_ERR("Failed to receive: %d", errno);
    }
    else if (ret > 0)
    {
      recv_buf[ret] = '\0';
      LOG_INF("Received %d bytes: %s", ret, recv_buf);
    }
    else
    {
      LOG_INF("Connection closed by server");
    }
  }
  else
  {
    // Use poll with manual timeout
    struct pollfd fds = {
        .fd     = sock,
        .events = POLLIN,
    };

    ret = poll(&fds, 1, SOCKET_TIMEOUT_SEC * 1000);

    if (ret < 0)
    {
      LOG_ERR("Poll error: %d", errno);
    }
    else if (ret == 0)
    {
      LOG_INF("Receive timeout");
    }
    else
    {
      ret = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);
      if (ret < 0)
      {
        LOG_ERR("Failed to receive data: %d", errno);
      }
      else if (ret > 0)
      {
        recv_buf[ret] = '\0';
        LOG_INF("Received %d bytes: %s", ret, recv_buf);
      }
      else
      {
        LOG_INF("Connection closed by server");
      }
    }
  }

  // Close socket
  close(sock);
  LOG_INF("TCP test on %s complete", iface_name);

  return 0;
}

/* Initialize modem */
static int init_modem(void)
{
  int err;

  LOG_INF("Initializing modem library");
  err = nrf_modem_lib_init();
  if (err)
  {
    LOG_ERR("Failed to initialize modem library: %d", err);
    return err;
  }

  LOG_INF("Connecting to LTE network asynchronously");
  err = lte_lc_connect_async(lte_handler);
  if (err)
  {
    LOG_ERR("Failed to initiate LTE connection: %d", err);
    return err;
  }

  LOG_INF("Waiting for LTE connection (%d seconds timeout)", MODEM_CONN_TIMEOUT_SEC);
  err = k_sem_take(&lte_connected, K_SECONDS(MODEM_CONN_TIMEOUT_SEC));
  if (err)
  {
    LOG_WRN("Timed out waiting for LTE connection");
    return -ETIMEDOUT;
  }

  LOG_INF("LTE connected successfully");
  return 0;
}

/* Print modem status information */
static void print_modem_status(void)
{
  int                       err;
  enum lte_lc_nw_reg_status reg_status;
  enum lte_lc_func_mode     func_mode;

  /* Get network registration status */
  err = lte_lc_nw_reg_status_get(&reg_status);
  if (err)
  {
    LOG_ERR("LTE: Failed to get network registration status: %d", err);
  }
  else
  {
    LOG_INF("LTE: Network registration status: %d", reg_status);
  }

  /* Get functional mode */
  err = lte_lc_func_mode_get(&func_mode);
  if (err)
  {
    LOG_ERR("Failed to get functional mode: %d", err);
  }
  else
  {
    LOG_INF("LTE: Functional mode: %d", func_mode);
  }

  /* Get system mode */
  enum lte_lc_system_mode            mode;
  enum lte_lc_system_mode_preference preference;

  err = lte_lc_system_mode_get(&mode, &preference);
  if (err)
  {
    LOG_ERR("LTE: Failed to get system mode: %d", err);
  }
  else
  {
    LOG_INF("LTE: System mode: %d, preference: %d", mode, preference);
  }
}

static void start_dhcpv4_client(struct net_if* iface, void* user_data)
{
  ARG_UNUSED(user_data);

  LOG_INF(
      "Starting DHCP on %s: index=%d", net_if_get_device(iface)->name, net_if_get_by_iface(iface));
  net_dhcpv4_start(iface);
}

static void print_dhcpv4_info(struct net_if* iface, int i)
{
  char buf[NET_IPV4_ADDR_LEN];

  LOG_INF("    Address[%d]: %s",
          net_if_get_by_iface(iface),
          net_addr_ntop(
              AF_INET, &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr, buf, sizeof(buf)));

  LOG_INF("    Subnet[%d]: %s",
          net_if_get_by_iface(iface),
          net_addr_ntop(AF_INET, &iface->config.ip.ipv4->unicast[i].netmask, buf, sizeof(buf)));

  LOG_INF("    Router[%d]: %s",
          net_if_get_by_iface(iface),
          net_addr_ntop(AF_INET, &iface->config.ip.ipv4->gw, buf, sizeof(buf)));

  if (iface->config.dhcpv4.lease_time)
  {
    LOG_INF(
        "Lease time[%d]: %u seconds", net_if_get_by_iface(iface), iface->config.dhcpv4.lease_time);
  }
}

static void
network_mgmt_handler(struct net_mgmt_event_callback* cb, uint32_t mgmt_event, struct net_if* iface)
{
  int i = 0;

  if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD)
  {
    return;
  }

  LOG_INF("IPv4 address added to interface %d", net_if_get_by_iface(iface));

  for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++)
  {
    if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP)
    {
      continue;
    }
    print_dhcpv4_info(iface, i);
  }
}

/* Run all tests for a given interface */
static void run_interface_tests(const char* iface_name)
{
  int err;

  LOG_INF("==========================================");
  LOG_INF("Starting tests for interface: %s", iface_name);
  LOG_INF("==========================================");

  /* Test socket binding */
  err = test_socket_binding(iface_name);
  if (err)
  {
    LOG_WRN("Socket binding test for %s failed: %d", iface_name, err);
    /* Continue with other tests anyway */
  }

  /* Test UDP communication */
  err = test_udp(iface_name);
  if (err)
  {
    LOG_WRN("UDP test for %s failed: %d", iface_name, err);
  }

  /* Test DNS resolution */
  err = test_dns_query(iface_name);
  if (err)
  {
    LOG_WRN("DNS test for %s failed: %d", iface_name, err);
  }

  /* Test TCP communication */
  err = test_tcp(iface_name);
  if (err)
  {
    LOG_WRN("TCP test for %s failed: %d", iface_name, err);
  }

  LOG_INF("==========================================");
  LOG_INF("Tests for interface %s completed", iface_name);
  LOG_INF("==========================================\n");
}

/* Test MQTT functionality on specified interface */
static int test_mqtt_interface(const char* iface_name)
{
  int      err;
  uint32_t message_counter = 0;

  LOG_INF("==========================================");
  LOG_INF("Starting MQTT test on interface: %s", iface_name);
  LOG_INF("==========================================");

  /* Test basic socket binding first */
  err = test_socket_binding(iface_name);
  if (err)
  {
    LOG_ERR("Socket binding test failed for %s: %d", iface_name, err);
    return err;
  }

  /* Connect to MQTT broker */
  err = mqtt_api_connect(iface_name);
  if (err)
  {
    LOG_ERR("Failed to connect to MQTT broker via %s: %d", iface_name, err);
    return err;
  }

  LOG_INF("MQTT connected successfully via %s", iface_name);

  /* Publish a few test messages */
  for (int i = 0; i < 3; i++)
  {
    err = mqtt_api_publish_status(++message_counter);
    if (err)
    {
      LOG_ERR("Failed to publish message %d: %d", message_counter, err);
    }
    else
    {
      LOG_INF("Published message %d successfully", message_counter);
    }

    /* Process MQTT input and keepalive */
    mqtt_api_process_input();
    mqtt_api_handle_keepalive();

    /* Wait between messages */
    k_sleep(K_SECONDS(5));
  }

  LOG_INF("==========================================");
  LOG_INF("MQTT test on interface %s completed", iface_name);
  LOG_INF("==========================================\n");

  return 0;
}

int main(void)
{
  int      err;
  uint32_t publish_counter   = 0;
  uint64_t next_publish_time = 0;

  LOG_INF("Socket Dispatcher Demo - Testing Multiple Network Interfaces with MQTT");
  k_msleep(1000);

  /* Initialize MQTT API */
  err = mqtt_api_init();
  if (err)
  {
    LOG_ERR("Failed to initialize MQTT API: %d", err);
    return err;
  }

  /* Setup DHCP for all interfaces */
  struct net_if* iface = net_if_get_default();
  net_mgmt_init_event_callback(&net_mgmt_cb, network_mgmt_handler, NET_EVENT_IPV4_ADDR_ADD);
  net_mgmt_add_event_callback(&net_mgmt_cb);
  start_dhcpv4_client(iface, NULL);

  /* Initialize modem */
  err = init_modem();
  if (err)
  {
    LOG_WRN("Modem initialization failed or timed out: %d", err);
    LOG_INF("Continuing with Ethernet interface only...");
  }
  else
  {
    /* Print modem status */
    print_modem_status();
  }

  /* Wait for interfaces to initialize and get IP addresses */
  LOG_INF("Waiting for interfaces to initialize...");
  k_sleep(K_SECONDS(5));

  // /* Run basic interface tests on cellular if available */
  // if (err == 0)
  // {
  //   run_interface_tests(IFACE_CELLULAR);
  // }

  // /* Run basic interface tests on Ethernet */
  // run_interface_tests(IFACE_ETHERNET);

  /* Test MQTT on Ethernet interface */
  LOG_INF("\n*** Starting MQTT Demo ***");
  err = test_mqtt_interface(IFACE_ETHERNET);
  if (err)
  {
    LOG_ERR("MQTT test failed on Ethernet interface: %d", err);
  }

  return 0;
}