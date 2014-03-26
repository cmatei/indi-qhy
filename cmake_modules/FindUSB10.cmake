# - Try to find LIBUSB
# Once done this will define
#
#  LIBUSB10_FOUND - system has libusb-1.0
#  LIBUSB10_INCLUDE_DIR - the libusb-1.0 include directory
#  LIBUSB10_LIBRARIES - Link these to use libusb-1.0

# Copyright (c) 2006, Jasem Mutlaq <mutlaqja@ikarustech.com>
# Based on FindLibfacile by Carsten Niehaus, <cniehaus@gmx.de>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (LIBUSB10_INCLUDE_DIR AND LIBUSB10_LIBRARIES)

  # in cache already
  set(LIBUSB10_FOUND TRUE)
  message(STATUS "Found LIBUSB10: ${LIBUSB10_LIBRARIES}")


else (LIBUSB10_INCLUDE_DIR AND LIBUSB10_LIBRARIES)

  find_path(LIBUSB10_INCLUDE_DIR libusb-1.0/libusb.h
    ${_obIncDir}
    ${GNUWIN32_DIR}/include
  )

  find_library(LIBUSB10_LIBRARIES NAMES usb-1.0
    PATHS
    ${_obLinkDir}
    ${GNUWIN32_DIR}/lib
  )

  if(LIBUSB10_INCLUDE_DIR AND LIBUSB10_LIBRARIES)
    set(LIBUSB10_FOUND TRUE)
  else (LIBUSB10_INCLUDE_DIR AND LIBUSB10_LIBRARIES)
    set(LIBUSB10_FOUND FALSE)
  endif(LIBUSB10_INCLUDE_DIR AND LIBUSB10_LIBRARIES)


  if (LIBUSB10_FOUND)
    if (NOT USB_FIND_QUIETLY)
      message(STATUS "Found LIBUSB10: ${LIBUSB10_LIBRARIES}")
    endif (NOT USB_FIND_QUIETLY)
  else (LIBUSB10_FOUND)
    if (USB_FIND_REQUIRED)
      message(FATAL_ERROR "LIBUSB10 not found. Please install libusb-devel and try again.")
    endif (USB_FIND_REQUIRED)
  endif (LIBUSB10_FOUND)
   
mark_as_advanced(LIBUSB10_INCLUDE_DIR LIBUSB10_LIBRARIES)

endif (LIBUSB10_INCLUDE_DIR AND LIBUSB10_LIBRARIES)
