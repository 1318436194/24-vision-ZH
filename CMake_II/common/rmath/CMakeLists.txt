aux_source_directory(src rmath_LIB)
add_library(rmath SHARED ${rmath_LIB})
target_include_directories(rmath PUBLIC include)