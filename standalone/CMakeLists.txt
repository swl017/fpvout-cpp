cmake_minimum_required(VERSION 3.1.0)
project(fpvLiberator)

include(FindPkgConfig)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(OpenCV REQUIRED)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 11)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h")

add_executable(${PROJECT_NAME} src/main.cpp ${HEADERS})
add_executable(${PROJECT_NAME}_pipe src/main_pipe.cpp ${HEADERS})
add_executable(${PROJECT_NAME}_shm src/main_shm.cpp ${HEADERS})
add_executable(${PROJECT_NAME}_classimpl src/main_classimpl.cpp ${HEADERS})

add_library(fpv_liberator_lib ${CMAKE_CURRENT_SOURCE_DIR}/../src/fpv_liberator.cpp ${SOURCES} ${HEADERS})
target_link_libraries(fpv_liberator_lib PkgConfig::libusb pthread rt ${OpenCV_LIBS})
target_include_directories(fpv_liberator_lib PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../include)

target_link_libraries(${PROJECT_NAME} PkgConfig::libusb)
target_link_libraries(${PROJECT_NAME}_pipe PkgConfig::libusb pthread)
target_link_libraries(${PROJECT_NAME}_shm PkgConfig::libusb pthread rt ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_classimpl fpv_liberator_lib PkgConfig::libusb pthread rt ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME}_pipe PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_classimpl PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/../src)