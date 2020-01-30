cmake_minimum_required(VERSION 3.13.4)
project(object_hunt_base)

set(CMAKE_CXX_STANDARD 14)

set(INCLUDE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SHARED_INCLUDE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/shared/include")

set(FUNDAMENTALS_HEADER
        ${INCLUDE_DIRECTORY}/GPIO.h
        ${INCLUDE_DIRECTORY}/GPIOException.h
        ${INCLUDE_DIRECTORY}/I2C.h
        ${INCLUDE_DIRECTORY}/I2CException.h
        )

set(SENSORS_HEADER
        ${INCLUDE_DIRECTORY}/HC_SR04.h
        ${INCLUDE_DIRECTORY}/TCST_1103.h
        ${INCLUDE_DIRECTORY}/TCST_1103Exception.h
        ${INCLUDE_DIRECTORY}/BNO_055.h
        ${INCLUDE_DIRECTORY}/BNO_055Exception.h
        )

set(ACTUATORS_HEADER
        ${INCLUDE_DIRECTORY}/VMA_401.h
        ${INCLUDE_DIRECTORY}/GearMotor.h
        ${INCLUDE_DIRECTORY}/VMA_401Exception.h
        ${INCLUDE_DIRECTORY}/GearMotorException.h
        )

set(OBJECT_HUNT_BASE_HEADER
        ${INCLUDE_DIRECTORY}/HardwareInterface.h
        ${INCLUDE_DIRECTORY}/HardwareInterfaceException.h
        ${INCLUDE_DIRECTORY}/TaskDistributor.h
        ${INCLUDE_DIRECTORY}/TaskDistributorException.h
        )

set(SHARED_HEADER
        ${SHARED_INCLUDE_DIRECTORY}/ObjectHuntDefinitions.h
        )

add_library(fundamentals
        src/GPIO.cpp
        src/I2C.cpp
        ${FUNDAMENTALS_HEADER}
)
target_include_directories(fundamentals PRIVATE
        ${INCLUDE_DIRECTORY})

add_library(sensors
        src/HC_SR04.cpp
        src/TCST_1103.cpp
        src/BNO_055.cpp
        ${SENSORS_HEADER}
)
target_include_directories(sensors PRIVATE
        ${INCLUDE_DIRECTORY})

add_library(actuators
        src/VMA_401.cpp
        src/GearMotor.cpp
        ${ACTUATORS_HEADER}
        ${SHARED_HEADER}
)
target_include_directories(actuators PRIVATE
        ${INCLUDE_DIRECTORY}
        ${SHARED_INCLUDE_DIRECTORY}
        )

add_executable(object_hunt_base
        src/ObjectHunterBase_main.cpp
        src/TaskDistributor.cpp
        src/HardwareInterface.cpp
        ${OBJECT_HUNT_BASE_HEADER}
        ${SHARED_HEADER}
)
target_include_directories(object_hunt_base PRIVATE
        ${INCLUDE_DIRECTORY}
        ${SHARED_INCLUDE_DIRECTORY}
        )

target_link_libraries(object_hunt_base
        sensors
        actuators
        fundamentals
        pthread
        atomic
)
