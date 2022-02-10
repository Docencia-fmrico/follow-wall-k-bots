# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_follow-wall_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED follow-wall_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(follow-wall_FOUND FALSE)
  elseif(NOT follow-wall_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(follow-wall_FOUND FALSE)
  endif()
  return()
endif()
set(_follow-wall_CONFIG_INCLUDED TRUE)

# output package information
if(NOT follow-wall_FIND_QUIETLY)
  message(STATUS "Found follow-wall: 0.0.0 (${follow-wall_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'follow-wall' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${follow-wall_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(follow-wall_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${follow-wall_DIR}/${_extra}")
endforeach()
