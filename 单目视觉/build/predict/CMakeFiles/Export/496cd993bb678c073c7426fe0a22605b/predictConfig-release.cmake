#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "predict::predict" for configuration "Release"
set_property(TARGET predict::predict APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(predict::predict PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpredict.so"
  IMPORTED_SONAME_RELEASE "libpredict.so"
  )

list(APPEND _cmake_import_check_targets predict::predict )
list(APPEND _cmake_import_check_files_for_predict::predict "${_IMPORT_PREFIX}/lib/libpredict.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
