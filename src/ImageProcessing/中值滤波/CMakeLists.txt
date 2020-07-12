CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
SET(PROJECT_NAME demo)
PROJECT(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 11)


FIND_PACKAGE(OpenCV REQUIRED)
if (OPENCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_definitions(-DENABLE_OPENCV)
    list(APPEND LINK_LIB_LIST ${OpenCV_LIBS})
endif (OPENCV_FOUND)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/
    )

#AUX_SOURCE_DIRECTORY(./ DIR_SRCS)
SET(SRC_DIR ${DIR_SRCS})


SET(EXTRA_LIBS 
    ${OpenCV_LIBS}
    )

file(GLOB example_files RELATIVE 
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    )
foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    add_executable(${example_name} ${example_file} ${SRC_DIR})
    target_link_libraries(${example_name} ${EXTRA_LIBS})
endforeach(example_file ${example_files})
