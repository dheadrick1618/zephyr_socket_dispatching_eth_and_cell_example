# Zephyr Socket Dispatching Example for Ethernet and Cellular Interface use in same project

The purpose of this project is to serve as an example of how to use an Ethernet and Cellular interface together on the nRF9151 SiP by Nordic Semi, running Zephyr RTOS and built using nRF Connect SDK.

I have created this because I was unable to find a good example anywhere online demonstrating how to use both a cellular network interface (facilitated by the nRF9151 internal modem), and an Ethernet interfae (facilitated by a Wiznet W5500), together in the same applicaiton.

## Application summary

To keep things consice and straighforward all the code in this project exists in the main.c source file.

## Why use Socket Dispatching?

The reason we must use Zephyr Socket Dispatching is as follows:

- 'Socket Offloading' is required for the application core of the nRF9151 to communicate with its internal cellular modem core. As mentioned in [this](https://devzone.nordicsemi.com/f/nordic-q-a/108023/aws_iot-example-without-network-offloading?pifragment-684=2) DevZone post by user Hakon, a member of their modem team.
- Socket offloading redirects (offloads) network traffic to specific BSD socket APIs. The modem lib features an 'integration layer'  which uses this offloading to direct all socket API calls to the modem library. Nordic does this to simplify the task of porting networking code to nRF91 series devices, although in our case it adds complications when your application requires additional interfaces which sockets may need to bind to and communicate with.
- The Ethernet interface in our application requires Native Sockets to be used so that the sockets assocaited with it, do not have their traffic directed to the nRF91 modem API.
- The socket() fxn does not allow the user to specify the specific network interface it should use, in the case both Native and Offloaded sockets are needed in the same application - this is our situation.
- Zephyr introduces a 'Socket Dispatcher' module, which allow socket creation to be postponed until the first socket operation is performed, allowing the user to set socket options (using setsockopt() fxn) to bind a particular socket to a desired network interface.

## Issues with Zephyr MQTT Client API

...

## Resources

BSD Sockets doc:

https://docs.zephyrproject.org/latest/connectivity/networking/api/sockets.html#dealing-with-multiple-offloaded-interfaces

Socket Dispatcher Kconfig doc:

https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_NET_SOCKETS_OFFLOAD_DISPATCHER
