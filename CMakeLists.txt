cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(zephyr_socket_dispatching_eth_and_cell_example)

target_sources(app PRIVATE 
    src/main.c
    src/mqtt_api.c
    src/mqtt_custom_transport.c
)