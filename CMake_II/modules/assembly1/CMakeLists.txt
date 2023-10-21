cmake_minimum_required(VERSION 3.10)
project(test)

aux_source_directory(src assembly1_LIB)
add_library(assembly1 ${assembly1_LIB})
target_include_directories(assembly1 PUBLIC include)

add_subdirectory(test)
enable_testing()
add_test(NAME assembly1_test COMMAND Test)