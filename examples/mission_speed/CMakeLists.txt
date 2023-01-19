cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mission_speed)

add_executable(mission_speed
    mission_speed.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(mission_speed
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(mission_speed PRIVATE -Wall -Wextra)
else()
    add_compile_options(mission_speed PRIVATE -WX -W2)
endif()
