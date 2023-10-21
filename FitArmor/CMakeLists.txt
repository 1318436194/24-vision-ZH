cmake_minimum_required(VERSION 3.10)
project(testCV)
find_package(OpenCV)

add_executable(main
        main.cpp
        fit.h
        init.cpp
        match.cpp
        feature.cpp)

target_link_libraries(main PUBLIC ${OpenCV_LIBS})
