aux_source_directory(src A2_LIB)
add_library(A2 SHARED ${A2_LIB})
target_include_directories(A2 PUBLIC include)