cmake_minimum_required(VERSION 3.10)
project(RC-CAR)

find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    "include"
)

# Print OpenCV details (for debugging)
message(STATUS "OpenCV_INCLWUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")


# Add source files
set(SOURCES
    ads.cpp
    lane-detection.cpp
)

set(HEADERS
    ads.hpp
)

set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)

# Create executable
add_executable(lane-detection ${SOURCES})

# Link OpenCV libraries
target_link_libraries(lane-detection ${OpenCV_LIBS})

# Ensure Windows finds DLLs at runtime
if(WIN32)
    set_target_properties(lane-detection PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/src/lane-detection/bin"
    )
endif()

# Print messages for debugging
message(STATUS "Lane-detection target configured.")