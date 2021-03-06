cmake_minimum_required(VERSION 2.6)
project (Dispatcher)
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

set (PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#Set QT
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Network)

set(PROJECT_SRCS
${PROJECT_SOURCE_DIR}/main.cpp ../src/MavState.cpp ../include/common/MavState.h)

include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_INCLUDE_DIR}")



add_executable(pub ${PROJECT_SRCS})

#Set LCM
#set ( PROJECT_LINK_LIBS liblcm.so )
#link_directories( ~/usr/local/lib )

find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)

## Declare a C++ library
 add_library(Dis
         libs/PositionDispatcher.cpp
         include/PositionDispatcher.h)

qt5_use_modules(pub Widgets Core Network SerialPort)
qt5_use_modules(Dis Widgets Core Network SerialPort)

target_link_libraries(pub
        ${PROJECT_LINK_LIBS}
        Qt5::Core
        Qt5::Network
        Qt5::Widgets
        ${LCM_LIBRARIES}
        Dis
        )

