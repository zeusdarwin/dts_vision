cmake_minimum_required(VERSION 3.10)
project(mindvision_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mv_camera_node.cpp
)
set(MINDVISION_SDK_ROOT "/home/darwin/MindvisionSDK")
set(MINDVISION_SDK_INCLUDE_DIR "${MINDVISION_SDK_ROOT}/include")
set(MINDVISION_SDK_LIBRARY "${MINDVISION_SDK_ROOT}/lib/amd64/libMVSDK.so")

if(NOT EXISTS "${MINDVISION_SDK_INCLUDE_DIR}/CameraApi.h")
  message(FATAL_ERROR "MindVision SDK include not found: ${MINDVISION_SDK_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${MINDVISION_SDK_LIBRARY}")
  message(FATAL_ERROR "MindVision SDK library not found: ${MINDVISION_SDK_LIBRARY}")
endif()

message(STATUS "Using MindVision SDK include: ${MINDVISION_SDK_INCLUDE_DIR}")
message(STATUS "Using MindVision SDK library: ${MINDVISION_SDK_LIBRARY}")

target_include_directories(${PROJECT_NAME} PUBLIC
  ${MINDVISION_SDK_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${MINDVISION_SDK_LIBRARY}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${MINDVISION_SDK_ROOT}/lib/amd64"
  INSTALL_RPATH "${MINDVISION_SDK_ROOT}/lib/amd64"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN mindvision_camera::MVCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  set(_linter_excludes
    mvsdk/include/CameraApi.h
    mvsdk/include/CameraDefine.h
    mvsdk/include/CameraStatus.h
  )
  ament_cpplint(EXCLUDE ${_linter_excludes})
endif()

# ############
# # Install ##
# ############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
