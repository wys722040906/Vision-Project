# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "predict/CMakeFiles/predict_autogen.dir/AutogenUsed.txt"
  "predict/CMakeFiles/predict_autogen.dir/ParseCache.txt"
  "predict/predict_autogen"
  )
endif()
