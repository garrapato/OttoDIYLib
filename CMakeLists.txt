cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME Setup)
project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_SKETCH OttoE_ScracthAI.ino)

#### Uncomment below additional settings as needed.
set(${CMAKE_PROJECT_NAME}_BOARD ATmega328P)
set(${CMAKE_PROJECT_NAME}_PORT /dev/cu.wchusbserial14310)
set(mega.build.mcu ATmega328P)
set(mega.upload.protocol wiring)
set(mega.upload.speed 115200)

#generate_arduino_firmware(${CMAKE_PROJECT_NAME})