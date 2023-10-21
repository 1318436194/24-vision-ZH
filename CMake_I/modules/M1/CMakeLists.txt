aux_source_directory(src M1_LIB)
add_library(M1 ${M1_LIB})
target_include_directories(M1 PUBLIC include ../A1/include)
target_link_libraries(M1 A1)