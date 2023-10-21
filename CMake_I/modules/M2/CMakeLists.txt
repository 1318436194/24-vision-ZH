aux_source_directory(src M2_LIB)
add_library(M2 SHARED ${M2_LIB})
target_include_directories(M2 PUBLIC include
        ../A1/include
        ../A2/include
        ../../common/kalman/include)
target_link_libraries(M2 A1 A2)