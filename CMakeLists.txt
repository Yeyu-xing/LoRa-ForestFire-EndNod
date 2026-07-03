cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME LoRa_SenLinFanghuo)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # User application sources
    ${CMAKE_SOURCE_DIR}/App/Src/app_config.c
    ${CMAKE_SOURCE_DIR}/App/Src/app_lora.c
    ${CMAKE_SOURCE_DIR}/App/Src/app_main.c
    ${CMAKE_SOURCE_DIR}/App/Src/app_menu.c
    ${CMAKE_SOURCE_DIR}/App/Src/board_io.c
    ${CMAKE_SOURCE_DIR}/App/Src/driver_button.c
    ${CMAKE_SOURCE_DIR}/App/Src/driver_co.c
    ${CMAKE_SOURCE_DIR}/App/Src/driver_eeprom.c
    ${CMAKE_SOURCE_DIR}/App/Src/driver_gnss.c
    ${CMAKE_SOURCE_DIR}/App/Src/driver_sht40.c
    ${CMAKE_SOURCE_DIR}/App/Src/lora_protocol.c
    ${CMAKE_SOURCE_DIR}/App/Src/menu_adapter.c
    ${CMAKE_SOURCE_DIR}/App/Src/test_button.c
    ${CMAKE_SOURCE_DIR}/App/Src/test_eeprom.c
    ${CMAKE_SOURCE_DIR}/App/Src/test_lora_simple.c
    ${CMAKE_SOURCE_DIR}/App/Src/test_sht40.c
    ${CMAKE_SOURCE_DIR}/App/Src/test_oled.c
    ${CMAKE_SOURCE_DIR}/App/Src/test_lora_rx.c
    
    
    # Easy_Menu library
    ${CMAKE_SOURCE_DIR}/Lib/Easy_Menu/Easy_Menu.c
    ${CMAKE_SOURCE_DIR}/Lib/Easy_Menu/Easy_Menu_Core.c
    ${CMAKE_SOURCE_DIR}/Lib/Easy_Menu/Easy_Menu_Item.c
    ${CMAKE_SOURCE_DIR}/Lib/Easy_Menu/Easy_Menu_Page.c
    ${CMAKE_SOURCE_DIR}/Lib/Easy_Menu/Easy_Menu_User.c
    
    # OLED driver library
    ${CMAKE_SOURCE_DIR}/Lib/STM32_HAL_SSD1306/oled.c
    ${CMAKE_SOURCE_DIR}/Lib/STM32_HAL_SSD1306/font.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User application headers
    ${CMAKE_SOURCE_DIR}/App/Inc
    
    # Third-party library headers
    ${CMAKE_SOURCE_DIR}/Lib/Easy_Menu
    ${CMAKE_SOURCE_DIR}/Lib/STM32_HAL_SSD1306
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
