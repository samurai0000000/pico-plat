# CMakeLists.txt
#
# Copyright (C) 2025, Charles Chiou

cmake_minimum_required(VERSION 3.0)

project(pico-plat)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -frtti")

add_compile_options(-Wall -Wextra -Werror)
add_compile_options(-g -O2 -I${CMAKE_CURRENT_LIST_DIR})

set(PICO_PLAT_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/serial.c
  ${CMAKE_CURRENT_SOURCE_DIR}/usbcdc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/PicoPlatform.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/PicoShell.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/pico-bme280/bme280.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Bme280.cxx
  )

add_library(pico-plat INTERFACE)
target_include_directories(pico-plat INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  )
target_sources(pico-plat INTERFACE ${PICO_PLAT_SRCS})
