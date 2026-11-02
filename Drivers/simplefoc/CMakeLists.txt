cmake_minimum_required(VERSION 3.22)
enable_language(C CXX)

message("Configuring SimpleFOC")

file(GLOB_RECURSE SIMPLEFOC_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/foc/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/foc/*.cpp"
)

add_library(simplefoc STATIC)

target_sources(simplefoc PRIVATE ${SIMPLEFOC_SOURCES})
target_include_directories(simplefoc PUBLIC
    # ${CMAKE_CURRENT_SOURCE_DIR}/drivers/encoders/mt6071
    ${CMAKE_CURRENT_SOURCE_DIR}/foc
    ${CMAKE_CURRENT_SOURCE_DIR}/foc/common
    ${CMAKE_CURRENT_SOURCE_DIR}/foc/common/base_classes
    ${CMAKE_CURRENT_SOURCE_DIR}/foc/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/foc/drivers/hardware_specific
    ${CMAKE_CURRENT_SOURCE_DIR}/foc/sensors
)

target_link_libraries(simplefoc PUBLIC stm32cubemx)
