cmake_minimum_required(VERSION 3.22)

project(ymir_box3d VERSION 0.1.0 LANGUAGES C)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX3D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_VALIDATE ON CACHE BOOL "" FORCE)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../extern/box3d" "${CMAKE_CURRENT_BINARY_DIR}/box3d")

add_library(ymir_box3d SHARED ymir_box3d.c)
target_link_libraries(ymir_box3d PRIVATE box3d::box3d)
target_include_directories(ymir_box3d PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../extern/box3d/include")
target_compile_features(ymir_box3d PRIVATE c_std_17)
if(MSVC)
    set_property(TARGET ymir_box3d PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

set_target_properties(ymir_box3d PROPERTIES
    C_VISIBILITY_PRESET hidden
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
