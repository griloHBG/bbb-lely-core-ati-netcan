cmake_minimum_required(VERSION 3.17)
project(lely_bbb)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_SYSROOT /home/grilo/gcc-linaro-6.3.1/sysroot-glibc-linaro-2.23-2017.05-arm-linux-gnueabihf)

set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER /home/grilo/gcc-linaro-6.3.1/gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /home/grilo/gcc-linaro-6.3.1/gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(ENV{PKG_CONFIG_PATH}  "$ENV{PKG_CONFIG_PATH}:/media/grilo/Data/Git/lely-core/install-arm/lib/pkgconfig")

find_package(PkgConfig REQUIRED)
#find_package(Threads REQUIRED)

pkg_check_modules(LELY-COAPP REQUIRED IMPORTED_TARGET liblely-coapp)


#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)

#set(CMAKE_EXE_LINKER_FLAGS "-static")

#set(CMAKE_CXX_FLAGS "--coverage")

#target_link_libraries(${PROJECT_NAME} Threads::Threads)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
add_executable(${PROJECT_NAME} main.cpp PID.cpp PID.h InteractionController.h)

#target_include_directories(${PROJECT_NAME} PRIVATE PkgConfig::LELY-IO2)

target_link_directories(${PROJECT_NAME} PRIVATE /media/grilo/Data/Git/lely-core/install-arm/lib)
target_link_libraries(${PROJECT_NAME} PRIVATE PkgConfig::LELY-COAPP)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic -g3)
target_include_directories(${PROJECT_NAME} PRIVATE json-develop/single_include/nlohmann)
#target_include_directories(${PROJECT_NAME} PRIVATE ./gnuplot-iostream)

#install(CODE "execute_process(COMMAND scp ${CMAKE_BINARY_DIR}/${PROJECT_NAME} debian@192.168.7.2:/home/debian/${PROJECT_NAME})")
