cmake_minimum_required(VERSION 3.1.0)
project(fpvLiberator)

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

add_executable(${PROJECT_NAME} src/main.cpp ${HEADERS})
add_executable(${PROJECT_NAME}_pipe src/main_pipe.cpp ${HEADERS})
add_executable(${PROJECT_NAME}_shm src/main_shm.cpp ${HEADERS})
add_executable(${PROJECT_NAME}_classimpl src/main_classimpl.cpp ${HEADERS})

add_library(fpv_liberator_lib ${CMAKE_CURRENT_SOURCE_DIR}/../src/fpv_liberator.cpp ${HEADERS})
target_link_libraries(fpv_liberator_lib PkgConfig::libusb pthread rt ${OpenCV_LIBS})
target_include_directories(fpv_liberator_lib PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_link_libraries(${PROJECT_NAME} PkgConfig::libusb)
target_link_libraries(${PROJECT_NAME}_pipe PkgConfig::libusb pthread)
target_link_libraries(${PROJECT_NAME}_shm PkgConfig::libusb PkgConfig::LIBAV pthread rt ${OpenCV_LIBS} ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY})
target_link_libraries(${PROJECT_NAME}_classimpl fpv_liberator_lib PkgConfig::libusb pthread rt ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME}_pipe PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_shm PRIVATE ${OpenCV_INCLUDE_DIRS} ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME}_classimpl PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/../src)