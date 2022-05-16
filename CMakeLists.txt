cmake_minimum_required (VERSION 3.0.0)
project (RADAR-MVP)

set(CMAKE_C_COMPILER /usr/bin/gcc-9)
set(CMAKE_CXX_COMPILER /usr/bin/g++-9)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/)

set(RP_API_SOURCE_DIR "${PROJECT_SOURCE_DIR}/external/RedPitaya/rp-api/api")

#if(NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release)
#endif()

set (CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -s -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

add_definitions (-DLINUX)
add_definitions (-DZ20)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_path(REDIS_PLUS_PLUS_HEADER sw)

message(STATUS "include dir: ${RP_API_SOURCE_DIR}")

#add_subdirectory("${RP_API_SOURCE_DIR}")

include_directories (
  ${RP_API_SOURCE_DIR}/include
  ${RP_API_SOURCE_DIR}/src/hw
  ${RP_API_SOURCE_DIR}/src/kiss_fft
  ${RP_API_SOURCE_DIR}/src
#  /opt/redpitaya/include
#  /usr/src/RedPitaya/api/src
)

file(GLOB_RECURSE rp_pa_sources ${RP_API_SOURCE_DIR}/src/*.c)
file(GLOB_RECURSE rp_pa_headers ${RP_API_SOURCE_DIR}/include/*.h)

#message(STATUS, "headers: ${rp_pa_headers}")

file (GLOB_RECURSE source RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/*.cpp)

find_package(NumCpp 2.6.2 REQUIRED)

find_package(Threads REQUIRED)

find_package(msgpack REQUIRED)

find_library(HIREDIS_LIBRARY hiredis)

find_library(REDIS_PLUS_PLUS_LIB redis++)

find_library(SERIAL_LIBRARY CppLinuxSerial)

add_executable(radar_redis_driver 
  ${RP_API_SOURCE_DIR}/src/common.c
  ${RP_API_SOURCE_DIR}/src/kiss_fft/kiss_fft.c
  ${RP_API_SOURCE_DIR}/src/kiss_fft/kiss_fftr.c
  ${RP_API_SOURCE_DIR}/src/oscilloscope.c 
  ${RP_API_SOURCE_DIR}/src/acq_handler.c 
  ${RP_API_SOURCE_DIR}/src/generate.c 
  ${RP_API_SOURCE_DIR}/src/gen_handler.c 
  ${RP_API_SOURCE_DIR}/src/spec_dsp.c 
  ${RP_API_SOURCE_DIR}/src/rp.c 
  ${RP_API_SOURCE_DIR}/src/hw/uart.c
  ${RP_API_SOURCE_DIR}/src/hw/led_system.c
  ${RP_API_SOURCE_DIR}/src/calib.c 

  ${PROJECT_SOURCE_DIR}/radar_redis_driver.cpp
)

set_target_properties(radar_redis_driver PROPERTIES CXX_STANDARD 17)

target_link_libraries(radar_redis_driver -lm -ludev -lpthread ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} ${SERIAL_LIBRARY} NumCpp::NumCpp Threads::Threads -lmsgpackc)
