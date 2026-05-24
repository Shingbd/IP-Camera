cmake_minimum_required(VERSION 3.16)
project(cam C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TARGET_SOC "rk356x")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_RTSP_SERVER REQUIRED gstreamer-rtsp-server-1.0)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ── RKNN model zoo (sets LIBRKNNRT, imageutils, etc.) ──
set(RKNN_ZOO ${CMAKE_CURRENT_SOURCE_DIR}/3rdpart/rknn_model_zoo)

add_subdirectory(${RKNN_ZOO}/3rdparty 3rdparty.out)
add_subdirectory(${RKNN_ZOO}/utils utils.out)

set(YOLOV8_POSE ${RKNN_ZOO}/examples/yolov8_pose/cpp)

# ── Sources ──
add_executable(cam
    cam.cpp
    ${YOLOV8_POSE}/postprocess.cc
    ${YOLOV8_POSE}/rknpu2/yolov8-pose.cc
)

target_include_directories(cam PRIVATE
    ${GST_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    ${GST_RTSP_SERVER_INCLUDE_DIRS}
    ${YOLOV8_POSE}
    ${RKNN_ZOO}/utils
    ${LIBRKNNRT_INCLUDES}
)

target_link_libraries(cam PRIVATE
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${GST_RTSP_SERVER_LIBRARIES}
    imageutils
    fileutils
    imagedrawing
    ${LIBRKNNRT}
    dl
    Threads::Threads
)

target_compile_options(cam PRIVATE
    ${GST_CFLAGS_OTHER}
    ${GST_APP_CFLAGS_OTHER}
    ${GST_RTSP_SERVER_CFLAGS_OTHER}
)
