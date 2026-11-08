cmake_minimum_required(VERSION 3.14)
project(dts_vision)
# Direct CMake builds need a writable default; colcon supplies its own prefix.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH
        "Install location for this build" FORCE)
endif()
option(DTS_BUILD_POINT_MSGS_ONLY "Internal build of the navigation message package" OFF)
if(DTS_BUILD_POINT_MSGS_ONLY)
    project(point_msgs)
    find_package(ament_cmake REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    # Keep source files in tools/rcl and generate the standard msg layout in build/.
    set(CMAKE_CURRENT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tools/rcl")
    ament_package_xml(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Message.msg" "${CMAKE_CURRENT_BINARY_DIR}/msg/Message.msg" COPYONLY)
    rosidl_generate_interfaces(${PROJECT_NAME}
        "${CMAKE_CURRENT_BINARY_DIR}:msg/Message.msg")
    ament_export_dependencies(rosidl_default_runtime)
    ament_package()
    return()
endif()


set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

#修改以下option需要rebuild清除缓存才能生效
option(BUILD_WITH_TRT      "Enable TensorRT backend"   OFF)
option(BUILD_WITH_OPENVINO "Enable OpenVINO backend"   ON)
option(BUILD_SENTRY "Enable Sentry"   ON)
option(BUILD_WITH_NAV_ROS2 "Enable sentry navigation communication via point_msgs" ON)
option(BUILD_WITH_PCH "Enable precompiled headers for C++ targets" ON)
option(BUILD_WITH_Daedalus "Enable Daedalus interface" OFF)
option(BUILD_WITH_RERUN "Enable Rerun visualization when the SDK is available" ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

set(DTSVISION_PCH_HEADER ${PROJECT_SOURCE_DIR}/tools/pch.hpp)
include(DtsVisionTargets)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(TBB REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)
find_package(HikSDK QUIET)
find_package(MvSDK QUIET)
set(BUILD_DEFINITIONS "")
set(BUILD_LIBS "")
if(HikSDK_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_HikSDK)
    list(APPEND BUILD_LIBS HikSDK::HikSDK)
endif()

if(MvSDK_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_MvSDK)
    list(APPEND BUILD_LIBS MvSDK::MvSDK)
endif()

set(RERUN_FOUND FALSE)
if(BUILD_WITH_RERUN)
    find_package(rerun_sdk CONFIG QUIET)
    if(TARGET rerun_sdk)
        set(RERUN_FOUND TRUE)
        message(STATUS "Rerun SDK found, enabling the Rerun visualization backend")
    else()
        message(WARNING "Rerun SDK not found; Web/ROS2 visualization remains available")
    endif()
endif()



if(RERUN_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_RERUN)
endif()
if(BUILD_WITH_Daedalus)
    list(APPEND BUILD_DEFINITIONS USE_Daedalus)
endif()

if(BUILD_WITH_OPENVINO)
    find_package(OpenVINO REQUIRED COMPONENTS Runtime ONNX)
    list(APPEND BUILD_DEFINITIONS USE_OPENVINO)
    list(APPEND BUILD_LIBS openvino::runtime openvino::frontend::onnx)
endif()

if(BUILD_WITH_TRT)
    find_package(CUDAToolkit REQUIRED)
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/darwin/TensorRT-10.6.0.26")
    find_package(TensorRT REQUIRED)
    include_directories(${TensorRT_INCLUDE_DIR})
    include_directories(/usr/local/cuda/include)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    set(CMAKE_CUDA_ARCHITECTURES 86)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)

    list(APPEND BUILD_DEFINITIONS USE_TRT)
    list(APPEND BUILD_LIBS TensorRT::TensorRT TensorRT::nvonnxparser CUDA::cudart  /usr/local/cuda/lib64/libnppicc.so)
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/param_deliver.h.in  
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/param_deliver.h            
    @ONLY
)

set(VIDEO_STREAM_PROTO
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks/auto_client/proto/video_stream.proto
)
protobuf_generate_cpp(VIDEO_STREAM_PROTO_SRCS VIDEO_STREAM_PROTO_HDRS
    ${VIDEO_STREAM_PROTO}
)
add_library(video_stream_proto STATIC
    ${VIDEO_STREAM_PROTO_SRCS}
    ${VIDEO_STREAM_PROTO_HDRS}
)
target_include_directories(video_stream_proto PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
)
if(TARGET protobuf::libprotobuf)
    target_link_libraries(video_stream_proto PUBLIC protobuf::libprotobuf)
else()
    target_include_directories(video_stream_proto PUBLIC ${Protobuf_INCLUDE_DIRS})
    target_link_libraries(video_stream_proto PUBLIC ${Protobuf_LIBRARIES})
endif()

dts_add_library(relink SOURCES tools/relink.cpp
    INCLUDES ${OpenCV_INCLUDE_DIRS}
    LINKS Eigen3::Eigen
)
set(TOOLS_SOURCE_GLOBS
    tools/*.cpp
    3rdparty/tinympc/*.cpp
    3rdparty/backward-cpp/backward.cpp
)
if(BUILD_WITH_TRT)
    list(APPEND TOOLS_SOURCE_GLOBS tools/*.cu)
endif()
dts_add_library(${PROJECT_NAME}_tools GLOB ${TOOLS_SOURCE_GLOBS}
    INCLUDES
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/3rdparty
        ${CMAKE_BINARY_DIR}/tools
        ${OpenCV_INCLUDE_DIRS}
    DEFINITIONS ${BUILD_DEFINITIONS}
    LINKS
    HikSDK::HikSDK
    MvSDK::MvSDK
    yaml-cpp
    fmt::fmt
    Eigen3::Eigen
    Ceres::ceres
    spdlog::spdlog
    TBB::tbb
    Boost::boost
    ${OpenCV_LIBS}
    ${BUILD_LIBS}
)
dts_add_library(${PROJECT_NAME}_net_detector GLOB tasks/base/net_detector/*.cpp LINKS ${PROJECT_NAME}_tools)
dts_add_library(${PROJECT_NAME}_auto_aim GLOB tasks/auto_aim/*.cpp LINKS ${PROJECT_NAME}_tools ${PROJECT_NAME}_net_detector)
dts_add_library(${PROJECT_NAME}_auto_buff GLOB tasks/auto_buff/*.cpp LINKS ${PROJECT_NAME}_tools ${PROJECT_NAME}_net_detector)
dts_add_library(${PROJECT_NAME}_auto_client GLOB tasks/auto_client/*.cpp LINKS
    ${PROJECT_NAME}_tools
    video_stream_proto
    PkgConfig::GSTREAMER
)
dts_add_library(${PROJECT_NAME}_web SOURCES tasks/base/web.cpp LINKS
    ${PROJECT_NAME}_auto_aim
    ${PROJECT_NAME}_auto_buff
)

if(RERUN_FOUND)
    # The distributed C++ SDK is static/non-PIC, so keep it at executable link scope.
    dts_add_library(${PROJECT_NAME}_rerun TYPE STATIC SOURCES tasks/rerun/recorder.cpp DEFINITIONS USE_RERUN
        LINKS rerun_sdk
        PRIVATE_LINKS ${PROJECT_NAME}_auto_aim ${PROJECT_NAME}_auto_buff
    )
endif()

set(STANDARD_LINKS ${PROJECT_NAME}_auto_aim ${PROJECT_NAME}_auto_buff ${PROJECT_NAME}_web)
set(HERO_LINKS ${PROJECT_NAME}_auto_aim ${PROJECT_NAME}_web ${PROJECT_NAME}_auto_client video_stream_proto)
set(SENTRY_LINKS ${PROJECT_NAME}_auto_aim ${PROJECT_NAME}_auto_buff ${PROJECT_NAME}_web)

dts_add_executable(hero SOURCES src/hero.cpp LINKS ${HERO_LINKS})
dts_add_executable(sentry SOURCES src/sentry.cpp LINKS ${SENTRY_LINKS})
dts_add_executable(standard SOURCES src/standard.cpp LINKS ${STANDARD_LINKS})
if(BUILD_WITH_NAV_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    option(DTS_BUNDLE_POINT_MSGS
        "Build the bundled point_msgs instead of reusing one from the environment" OFF)
    # Prefer a point_msgs already provided by the environment (the navigation
    # workspace ships one). Linking our own copy while another is on the runtime
    # library path registers two typesupport sets for the same type name, and the
    # publisher then never makes it into the DDS graph.
    if(DEFINED CACHE{point_msgs_DIR})
        string(FIND "${point_msgs_DIR}" "${CMAKE_INSTALL_PREFIX}" DTS_POINT_MSGS_CACHED_LOCAL)
        if(DTS_POINT_MSGS_CACHED_LOCAL EQUAL 0)
            # Drop a bundled path cached by an earlier configure so the probe below
            # can still see an environment-provided package.
            unset(point_msgs_DIR CACHE)
        endif()
    endif()
    if(NOT DTS_BUNDLE_POINT_MSGS)
        find_package(point_msgs QUIET CONFIG)
    endif()
endif()
if(BUILD_WITH_NAV_ROS2 AND point_msgs_FOUND)
    message(STATUS "point_msgs: reusing ${point_msgs_DIR}")
endif()
if(BUILD_WITH_NAV_ROS2 AND NOT point_msgs_FOUND)
    message(WARNING
        "point_msgs was not found in the environment, so a bundled copy will be built "
        "and installed into ${CMAKE_INSTALL_PREFIX}. If the navigation stack provides "
        "its own point_msgs at runtime, both will be loaded and /auto_aim_plan will not "
        "be published. Source that workspace before configuring, or pass -Dpoint_msgs_DIR=.")
    # Build the message package using this same file before importing its targets.
    # Share the install prefix so colcon can discover its runtime environment.
    set(POINT_MSGS_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/point_msgs_build")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        tools/rcl/Message.msg tools/rcl/package.xml)
    execute_process(COMMAND "${CMAKE_COMMAND}"
        -S "${CMAKE_CURRENT_SOURCE_DIR}" -B "${POINT_MSGS_BUILD_DIR}"
        -G "${CMAKE_GENERATOR}"
        -DDTS_BUILD_POINT_MSGS_ONLY=ON
        "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
        "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
        "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
        "-DPython3_EXECUTABLE=${Python3_EXECUTABLE}"
        RESULT_VARIABLE POINT_MSGS_CONFIGURE_RESULT)
    if(NOT POINT_MSGS_CONFIGURE_RESULT EQUAL 0)
        message(FATAL_ERROR "point_msgs configuration failed")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" --build "${POINT_MSGS_BUILD_DIR}"
        --target install --parallel 2
        RESULT_VARIABLE POINT_MSGS_BUILD_RESULT)
    if(NOT POINT_MSGS_BUILD_RESULT EQUAL 0)
        message(FATAL_ERROR "point_msgs build/install failed")
    endif()
    install(SCRIPT "${POINT_MSGS_BUILD_DIR}/cmake_install.cmake")
    set(point_msgs_DIR "${CMAKE_INSTALL_PREFIX}/share/point_msgs/cmake"
        CACHE PATH "Bundled point_msgs package configuration" FORCE)
    find_package(point_msgs CONFIG REQUIRED)
endif()
if(BUILD_WITH_NAV_ROS2)
    target_compile_definitions(sentry PRIVATE USE_NAV_ROS2)
    ament_target_dependencies(sentry rclcpp point_msgs)
endif()
