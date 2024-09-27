cmake_minimum_required (VERSION 3.8)

project("PowerDiagramGenerator")

set(PUBLISH_BIN_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/build/bin/debug)
set(PUBLISH_BIN_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/build/bin/release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PUBLISH_BIN_DEBUG})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PUBLISH_BIN_RELEASE})
add_compile_definitions(OUTPUT_PATH="${PROJECT_SOURCE_DIR}/data/")
add_compile_options(/openmp:llvm)

include_directories(${PROJECT_SOURCE_DIR}/include) 

set(SRC_DIRS ${PROJECT_SOURCE_DIR}/src)
aux_source_directory(${SRC_DIRS} SRC_SOURCE)
add_executable(PowerDiagramGenerator ${SRC_SOURCE})