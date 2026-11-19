cmake_minimum_required(VERSION 3.22)

project(ymir_box3d VERSION 0.1.0 LANGUAGES C)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX3D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_VALIDATE ON CACHE BOOL "" FORCE)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../extern/box3d" "${CMAKE_CURRENT_BINARY_DIR}/box3d")

add_library(ymir_box3d SHARED ymir_box3d.c ymir_box3d_soa.c)
target_link_libraries(ymir_box3d PRIVATE box3d::box3d)
target_include_directories(ymir_box3d PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../extern/box3d/include")
target_compile_features(ymir_box3d PRIVATE c_std_17)
if(MSVC)
    set_property(TARGET ymir_box3d PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

set_target_properties(ymir_box3d PROPERTIES
    C_VISIBILITY_PRESET hidden
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
if(MINGW)
    # MinGW would emit libymir_box3d.dll; the managed side loads ymir_box3d.dll.
    set_property(TARGET ymir_box3d PROPERTY PREFIX "")
endif()
