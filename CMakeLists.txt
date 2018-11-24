cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi_cam84_ccd C CXX)

option(LIBFTDI "Use the LIBFTDI Library" OFF)

if(LIBFTDI)
	add_definitions(-DLIBFTDI)
endif()

SET ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread" )
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_modules/")

include(GNUInstallDirs)

set (CAM84_VERSION_MAJOR 1)
set (CAM84_VERSION_MINOR 0)

add_library(cam84 libcam84.cpp)

find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)
find_package ( LibFTDI1 NO_MODULE REQUIRED )
include ( ${LIBFTDI_USE_FILE} )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_cam84_ccd.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_cam84_ccd.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
#include_directories ( ${LIBFTDI_USE_FILE} )
#include_directories(${LIBFTD2XX_INCLUDE_DIR})

include_directories( ${CFITSIO_INCLUDE_DIR})

############# CAM84 CCD ###############
if (CFITSIO_FOUND)

set(cam84ccd_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/cam84_ccd.cpp
)

add_executable(indi_cam84_ccd ${cam84ccd_SRCS})

target_link_libraries(indi_cam84_ccd cam84 /usr/local/lib/libftd2xx.so ${LIBFTDI_LIBRARIES} ${INDI_DRIVER_LIBRARIES} ${CFITSIO_LIBRARIES} m ${ZLIB_LIBRARY})

install(TARGETS indi_cam84_ccd RUNTIME DESTINATION bin)

endif (CFITSIO_FOUND)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_cam84_ccd.xml DESTINATION ${INDI_DATA_DIR})
