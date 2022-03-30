cmake_minimum_required (VERSION 3.0.0)
project (RADAR-MVP)

set(CMAKE_C_COMPILER /usr/bin/gcc-9)
set(CMAKE_CXX_COMPILER /usr/bin/g++-9)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions (-DLINUX)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_path(REDIS_PLUS_PLUS_HEADER sw)

include_directories (
  /opt/redpitaya/include
)

file (GLOB_RECURSE source RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/*.cpp)

find_package(Threads REQUIRED)

find_package(msgpack REQUIRED)

find_library(HIREDIS_LIBRARY hiredis)

find_library(REDIS_PLUS_PLUS_LIB redis++)

find_library(SERIAL_LIBRARY CppLinuxSerial)

add_executable(radar_redis_driver ${PROJECT_SOURCE_DIR}/radar_redis_driver.cpp)

set_target_properties(radar_redis_driver PROPERTIES CXX_STANDARD 17)

target_link_libraries(radar_redis_driver -L/opt/redpitaya/lib -lrp -lm -ludev -lpthread ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} ${SERIAL_LIBRARY} Threads::Threads -lmsgpackc)
