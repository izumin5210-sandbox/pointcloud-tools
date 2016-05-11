project(tools)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_PREFIX_PATH} -W -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_PREFIX_PATH} -march=native")

if (APPLE)
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Cellar/opencv3/3.1.0/share")
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Cellar/qt5/5.5.1_2/lib/cmake")
endif ()

find_package        (OpenCV 3.0 REQUIRED)
find_package        (Boost COMPONENTS system filesystem program_options thread REQUIRED)
find_package        (PCL 1.8 REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(TOOLS_SOURCE_DIR ${PROJECT_SOURCE_DIR}/tools)
set(UTILS_SOURCE_DIR ${PROJECT_SOURCE_DIR}/utils)

file(GLOB exec_src RELATIVE "${TOOLS_SOURCE_DIR}" "**/*.cpp")

include_directories(
    "${PROJECT_SOURCE_DIR}"
)

foreach (cppfile ${exec_src})
  string(REGEX REPLACE ".cpp\$" "" out ${cppfile})
  message(STATUS "add: ${out} ${cppfile}")
  add_executable(${out} "${TOOLS_SOURCE_DIR}/${cppfile}")
  target_link_libraries(${out}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBS}
      ${Boost_LIBRARIES}
      freenect2
      yaml-cpp
  )
endforeach (cppfile)
