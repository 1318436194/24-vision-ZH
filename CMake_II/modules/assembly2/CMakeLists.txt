aux_source_directory(src assembly2_LIB)
add_library(assembly2 ${assembly2_LIB})
target_include_directories(assembly2 PUBLIC include)
