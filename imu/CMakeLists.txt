cmake_minimum_required(VERSION 3.20)

project(bno085_imu_cpp VERSION 0.1.0 LANGUAGES CXX)

# C++20, dość surowe flagi
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall -Wextra -Wpedantic
        -Wconversion -Wsign-conversion
        -Wshadow -Wnon-virtual-dtor
        -Wold-style-cast -Wcast-align
        -Woverloaded-virtual
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
        -Wimplicit-fallthrough
        -Wmissing-include-dirs
        -Wmissing-declarations
        -Wmissing-noreturn
        -Wswitch-enum
        -fno-exceptions
    )
    # Treat warnings as errors w domyśle (możesz wyłączyć przez -DWARNINGS_AS_ERRORS=OFF)
    option(WARNINGS_AS_ERRORS "Treat warnings as errors" ON)
    if (WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

# Opcjonalne zależności – na razie nie wymagamy ich twardo
find_package(spdlog QUIET)

add_library(libbno_shtp
    src/shtp_linux_i2c.cpp
    src/sh2_parser.cpp
)

target_include_directories(libbno_shtp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Jeśli jest spdlog, dołącz jako interfejs
if (spdlog_FOUND)
    target_compile_definitions(libbno_shtp PUBLIC HAVE_SPDLOG=1)
    target_link_libraries(libbno_shtp PUBLIC spdlog::spdlog)
endif()

# Potrzebne nagłówki systemowe dla i2c-dev
if (UNIX)
    target_compile_definitions(libbno_shtp PUBLIC BNO_LINUX_I2C=1)
endif()

# --- imu_read_cpp ---

add_executable(imu_read
    src/imu_read.cpp
)

target_link_libraries(imu_read
    PRIVATE
        libbno_shtp
)

# --- imu_status_cpp ---

add_executable(imu_status
    src/imu_status.cpp
)

target_link_libraries(imu_status
    PRIVATE
        libbno_shtp
)

add_executable(imu_dir
    src/imu_dir.cpp
    src/shtp_linux_i2c.cpp
    src/sh2_parser.cpp
)

target_include_directories(imu_dir PRIVATE include)

# Jeżeli masz wspólną bibliotekę typu libbno_shtp, podlinkuj ją zamiast powtarzać pliki:
# target_link_libraries(imu_dir_cpp PRIVATE bno_shtp)

# Przyjazne wyjście
message(STATUS "Configured targets: imu_read, imu_status, libbno_shtp")
