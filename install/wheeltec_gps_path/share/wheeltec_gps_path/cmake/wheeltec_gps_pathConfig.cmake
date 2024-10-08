# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wheeltec_gps_path_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wheeltec_gps_path_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wheeltec_gps_path_FOUND FALSE)
  elseif(NOT wheeltec_gps_path_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wheeltec_gps_path_FOUND FALSE)
  endif()
  return()
endif()
set(_wheeltec_gps_path_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wheeltec_gps_path_FIND_QUIETLY)
  message(STATUS "Found wheeltec_gps_path: 0.0.0 (${wheeltec_gps_path_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wheeltec_gps_path' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wheeltec_gps_path_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wheeltec_gps_path_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wheeltec_gps_path_DIR}/${_extra}")
endforeach()
