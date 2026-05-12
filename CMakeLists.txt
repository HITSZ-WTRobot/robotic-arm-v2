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
set(CMAKE_PROJECT_NAME robotic-arm-v2)

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
    # Add user sources here
    UserCode/app.cpp
    UserCode/armctrl.cpp
    UserCode/arena.cpp
    UserCode/device.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add dependence from library
# ===================== DEPENDENCIES =====================
# e.g.
#add_subdirectory(library/motor_drivers/UserCode)
add_subdirectory(Modules/BasicComponents)
add_subdirectory(Modules/MotorDrivers)
# add_subdirectory(Modules/ChassisController)
add_subdirectory(Modules/VelocityProfile)
add_subdirectory(Modules/TrajectoryControl)
add_subdirectory(Modules/ArmController)
add_subdirectory(Modules/Sensors)

# =======================================================

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
        bsp::CANDriver
        MotorDrivers::DJI
        MotorDrivers::DM
        SensorGyro::HWT101CT
        SensorOPS::ActionOPS
        SensorLaser::STP23L
        SensorLaser::DT35
        SensorPressure::XGZP6847D
        services::I2CUpdateManager
        utils
        # Chassis::Mecanum4
        ArmController
        ArmController::Slave
)
