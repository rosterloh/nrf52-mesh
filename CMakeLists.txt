cmake_minimum_required(VERSION 3.8.2)

set(BOARD nrf52_adafruit_feather)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(nrf52_mesh)

# target_link_libraries(app PUBLIC subsys__bluetooth)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})