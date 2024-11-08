# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fake_sensors_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fake_sensors_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fake_sensors_FOUND FALSE)
  elseif(NOT fake_sensors_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fake_sensors_FOUND FALSE)
  endif()
  return()
endif()
set(_fake_sensors_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fake_sensors_FIND_QUIETLY)
  message(STATUS "Found fake_sensors: 0.0.0 (${fake_sensors_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fake_sensors' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT fake_sensors_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fake_sensors_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fake_sensors_DIR}/${_extra}")
endforeach()
