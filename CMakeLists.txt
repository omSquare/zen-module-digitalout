cmake_minimum_required(VERSION 3.8.2)

set(BOARD "atsamd20_xpro")
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(zen-module-digitalout C)

set(CMAKE_C_STANDARD 11)
target_sources(app PRIVATE src/main.c src/zbus.h)
