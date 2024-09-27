cmake_minimum_required (VERSION 3.8)

project("PowerDiagramGenerator")

add_compile_definitions(OUTPUT_PATH="${PROJECT_SOURCE_DIR}/data/")
add_compile_options(/openmp:llvm)

include_directories(${PROJECT_SOURCE_DIR}/include) 

set(SRC_DIRS ${PROJECT_SOURCE_DIR}/src)
aux_source_directory(${SRC_DIRS} SRC_SOURCE)
add_executable(PowerDiagramGenerator ${SRC_SOURCE})