aux_source_directory(src math_LIB)
add_library(math SHARED ${math_LIB})
target_include_directories(math PUBLIC include)