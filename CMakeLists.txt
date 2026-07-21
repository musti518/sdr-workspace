cmake_minimum_required(VERSION 3.16)
project(sdr-workspace C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(libvrt)

find_package(Boost 1.65 REQUIRED COMPONENTS
  program_options chrono filesystem thread date_time system)
find_path(ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_library(ZMQ_LIBRARY zmq REQUIRED)
find_package(Threads REQUIRED)

add_executable(iq_recorder src/iq_recorder.cpp)
find_package(UHD QUIET)
if(${UHD_FOUND})
  add_executable(usrp_to_vrt src/usrp_to_vrt.cpp)
  target_include_directories(usrp_to_vrt PRIVATE ${UHD_INCLUDE_DIRS})
  target_link_libraries(usrp_to_vrt PRIVATE ${UHD_LIBRARIES})
else()
  message(WARNING "UHD not found, not building usrp_to_vrt")
endif()
# Weitere Testprogramme hier eintragen, sobald sie entstehen:
# add_executable(test_source_gnss src/test_source_gnss.cpp)
add_executable(vrt_to_rtl_tcp src/vrt_to_rtl_tcp.cpp)
add_executable(gnss_to_vrt src/gnss_to_vrt.cpp)
get_property(all_targets DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  PROPERTY BUILDSYSTEM_TARGETS)

foreach(target ${all_targets})
  if(target STREQUAL "vrt")
    continue()
  endif()
  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libvrt/include
    ${ZMQ_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS})
  target_link_libraries(${target} PRIVATE
    vrt ${ZMQ_LIBRARY} ${Boost_LIBRARIES} Threads::Threads)
endforeach()
