cmake_minimum_required(VERSION 3.13)
project(object_hunt_navigation)

set(CMAKE_CXX_STANDARD 14)

set(INCLUDE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SHARED_INCLUDE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../object_hunt_base/shared/include")

set(OBJECT_HUNT_NAVIGATION_HEADER
        ${INCLUDE_DIRECTORY}/Navigator.h
        ${INCLUDE_DIRECTORY}/ConnectionManager.h
        ${INCLUDE_DIRECTORY}/NavigatorException.h
        )

set(SHARED_HEADER
        ${SHARED_INCLUDE_DIRECTORY}/ObjectHuntDefinitions.h
        )

add_executable(object_hunt_navigation
        src/ObjectHunterNavigation_main.cpp
        src/Navigator.cpp
        src/ConnectionManager.cpp
        ${OBJECT_HUNT_NAVIGATION_HEADER}
        ${SHARED_HEADER}
        )

target_include_directories(object_hunt_navigation PRIVATE
        ${INCLUDE_DIRECTORY}
        ${SHARED_INCLUDE_DIRECTORY}
        )

target_link_libraries(object_hunt_navigation
        pthread)
