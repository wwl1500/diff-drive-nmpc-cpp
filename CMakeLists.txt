cmake_minimum_required(VERSION 3.16)
project(diff_drive_nmpc_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(diff_drive_core
    src/diff_drive_model.cpp
    src/reference_generator.cpp
    src/pure_pursuit_controller.cpp
    src/csv_logger.cpp
)

target_include_directories(diff_drive_core PUBLIC include)

target_compile_options(diff_drive_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(diff_drive_sim src/main.cpp)
target_link_libraries(diff_drive_sim PRIVATE diff_drive_core)

target_compile_options(diff_drive_sim PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
