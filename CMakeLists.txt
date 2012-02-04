cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi_qhy CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")
set(FIRMWARE_INSTALL_DIR "/lib/firmware")
set(UDEVRULES_INSTALL_DIR "/etc/udev/rules.d")
SET(CMAKE_C_FLAGS "-Wall -g -O0" )
SET(CMAKE_C_FLAGS_DEBUG "-Werror" )

SET(CMAKE_CXX_FLAGS "-Wall -g -O0" )
SET(CMAKE_CXX_FLAGS_DEBUG "-Werror" )

Include (CheckCSourceCompiles)
include (MacroOptionalFindPackage)
include (MacroLogFeature)
include (MacroBoolTo01)
include (CheckIncludeFiles)

find_package(USB10 REQUIRED)
find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)

#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${CFITSIO_INCLUDE_DIR})

########### QHY ###########
set(indiqhy_SRCS
  ${CMAKE_SOURCE_DIR}/qhyindi.cc
  ${CMAKE_SOURCE_DIR}/qhyccd.cc
  ${CMAKE_SOURCE_DIR}/qhy9.cc
  ${CMAKE_SOURCE_DIR}/qhy5.cc
  )

add_executable(indi_qhy ${indiqhy_SRCS})

target_link_libraries(indi_qhy ${INDI_LIBRARIES} ${INDI_MAIN_LIBRARIES} ${INDI_DRIVER_LIBRARIES}
  ${CFITSIO_LIBRARIES} ${QHY_LIBRARIES} ${LIBUSB10_LIBRARIES} ${ZLIB_LIBRARIES} )

install(TARGETS indi_qhy RUNTIME DESTINATION bin )

install(FILES indi_qhy.xml DESTINATION ${INDI_DATA_DIR} )

install(FILES firmware/qhy5.hex DESTINATION ${FIRMWARE_INSTALL_DIR})
install(FILES firmware/qhy5loader.hex DESTINATION ${FIRMWARE_INSTALL_DIR})
install(FILES firmware/85-qhy.rules DESTINATION ${UDEVRULES_INSTALL_DIR})
