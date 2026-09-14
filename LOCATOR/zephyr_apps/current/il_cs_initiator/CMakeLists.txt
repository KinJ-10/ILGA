# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(il_cs_initiator)

set(CS_SAMPLE_DIR ${ZEPHYR_BASE}/samples/bluetooth/channel_sounding)

target_sources(app PRIVATE
  src/main.c
  ${CS_SAMPLE_DIR}/src/distance_estimation.c
)

zephyr_library_include_directories(${CS_SAMPLE_DIR}/include)
