cmake_minimum_required(VERSION 3.13.1)

#set(BOARD "atsamd20_xpro")
#set(BOARD "native_posix")

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(zen-module-digitalout C)

set(CMAKE_C_STANDARD 11)
target_sources(app PRIVATE src/zbus.h)
target_sources_ifdef(CONFIG_SOC_FAMILY_SAM0 app PRIVATE src/main_sam0.c src/zbus_sam0.h src/zbus_sam0.c)
target_sources_ifdef(CONFIG_BOARD_NATIVE_POSIX app PRIVATE src/main.c src/zbus_native_posix.c)
