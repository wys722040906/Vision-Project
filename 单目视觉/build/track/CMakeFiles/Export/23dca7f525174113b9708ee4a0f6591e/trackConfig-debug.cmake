#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "track::track" for configuration "Debug"
set_property(TARGET track::track APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(track::track PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libtrack.so"
  IMPORTED_SONAME_DEBUG "libtrack.so"
  )

list(APPEND _cmake_import_check_targets track::track )
list(APPEND _cmake_import_check_files_for_track::track "${_IMPORT_PREFIX}/lib/libtrack.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
