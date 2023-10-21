aux_source_directory(src A1_LIB)
add_library(A1 SHARED ${A1_LIB})
target_include_directories(A1 PUBLIC include)

