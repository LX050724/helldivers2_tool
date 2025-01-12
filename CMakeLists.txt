cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H750_APP)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 全局头文件
include_directories(include)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(LV_CONF_BUILD_DISABLE_EXAMPLES YES)
set(LV_CONF_BUILD_DISABLE_DEMOS YES)
set(LV_CONF_BUILD_DISABLE_THORVG_INTERNAL YES)
add_subdirectory(Third_Party/lvgl)
get_target_property(MCU_INCLUDE stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(lvgl PUBLIC ${MCU_INCLUDE})

add_subdirectory(images)

add_subdirectory(Third_Party/SEGGER_RTT_V812)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/GT9147_driver.c
    Core/Src/lvgl_driver.c
    Core/Src/lvgl_app.c
    Core/Src/app_main.c
    Core/Src/helldivers.c
    Core/Src/lv_font_simhei_16.c
    Core/Src/lv_font_simhei_12.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    SEGGER_RTT
    lvgl
    lvgl_images
)

# target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -N)
# target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -mlong-calls)

add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME} PRE_BUILD  
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/userfile/STM32H750XBHx_FLASH.ld ${CMAKE_SOURCE_DIR}/STM32H750XBHx_FLASH.ld
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/userfile/usbd_hid.c ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/userfile/usbd_hid.h ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc/usbd_hid.h
)

