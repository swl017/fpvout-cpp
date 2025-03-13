cmake_minimum_required(VERSION 3.1.0)
project(fpvout_standalone)

include(FindPkgConfig)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavdevice
    libavfilter
    libavformat
    libavcodec
    libswresample
    libswscale
    libavutil
)
find_package(OpenCV REQUIRED)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)
find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)
find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)
include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 11)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h")

# fpvout library
add_library(libfpvout src/fpvout.cpp)
target_link_libraries(libfpvout PkgConfig::libusb PkgConfig::LIBAV pthread rt ${OpenCV_LIBS} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY})
target_include_directories(libfpvout PRIVATE ${OpenCV_INCLUDE_DIRS} ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR})

# Custom app example. Link with libfpvout
add_executable(fpvout_example src/fpvout_main.cpp)
target_link_libraries(fpvout_example libfpvout)
target_include_directories(fpvout_example PRIVATE ${OpenCV_INCLUDE_DIRS} ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR})