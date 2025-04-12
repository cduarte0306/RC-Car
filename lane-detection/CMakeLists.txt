cmake_minimum_required(VERSION 3.10)
project(RC-CAR-lane-detection)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    "include"
)

# Print OpenCV details (for debugging)
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

# Add source files
set(SOURCES
    src/adas.cpp
    src/lane-detection.cpp
    src/twinlitenet_dnn.cpp
)

set(HEADERS
    include/adas.hpp
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

    # Copy OpenCV DLLs to the output directory
    add_custom_command(TARGET lane-detection POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${OpenCV_DIR}/x64/vc15/bin/opencv_world412.dll
        $<TARGET_FILE_DIR:lane-detection>
    )
endif()

# Print messages for debugging
message(STATUS "Lane-detection target configured.")