cmake_minimum_required (VERSION 3.8)

project("PowerDiagramGenerator")

# set publish bin location
set(PUBLISH_BIN_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/build/bin/debug)
set(PUBLISH_BIN_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/build/bin/release)
# set runtime output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PUBLISH_BIN_DEBUG})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PUBLISH_BIN_RELEASE})
# set output file directory
add_compile_definitions(OUTPUT_PATH="${PROJECT_SOURCE_DIR}/data/")

# find OpenMP (not necessary)
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# add libraries directory
include_directories(${PROJECT_SOURCE_DIR}/include) 
include_directories(${PROJECT_SOURCE_DIR}/external) 
# add src files
set(SRC_DIRS ${PROJECT_SOURCE_DIR}/src)
aux_source_directory(${SRC_DIRS} SRC_SOURCE)

add_executable(PowerDiagramGenerator ${SRC_SOURCE})

# link to OpenMP library
if (OPENMP_FOUND)
    target_link_libraries(PowerDiagramGenerator ${OpenMP_CXX_FLAGS})
endif()