# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_body_follow_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED body_follow_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(body_follow_FOUND FALSE)
  elseif(NOT body_follow_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(body_follow_FOUND FALSE)
  endif()
  return()
endif()
set(_body_follow_CONFIG_INCLUDED TRUE)

# output package information
if(NOT body_follow_FIND_QUIETLY)
  message(STATUS "Found body_follow: 0.0.0 (${body_follow_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'body_follow' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${body_follow_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(body_follow_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${body_follow_DIR}/${_extra}")
endforeach()
