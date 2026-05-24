cmake_minimum_required(VERSION 3.16)
project(cam CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(OPENCV REQUIRED opencv4)
pkg_check_modules(GST_RTSP_SERVER REQUIRED gstreamer-rtsp-server-1.0)

add_executable(cam cam.cpp)
target_link_libraries(cam PRIVATE
    ${GST_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${OPENCV_LIBRARIES}
    ${GST_RTSP_SERVER_LIBRARIES}
    )
target_include_directories(cam PRIVATE
    ${GST_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
    ${GST_RTSP_SERVER_INCLUDE_DIRS}
    )
target_compile_options(cam PRIVATE
    ${GST_CFLAGS_OTHER}
    ${GST_APP_CFLAGS_OTHER}
    ${OPENCV_CFLAGS_OTHER}
    ${GST_RTSP_SERVER_CFLAGS_OTHER}
    )
