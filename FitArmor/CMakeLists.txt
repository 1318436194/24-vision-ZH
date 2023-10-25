cmake_minimum_required(VERSION 3.10)
project(testCV)
find_package(OpenCV)

add_library(fit include/init.cpp include/feature.cpp include/match.cpp)
target_include_directories(fit PUBLIC include)

add_executable(main main.cpp)
add_executable(Armors_fit Armors_fit.cpp)

target_link_libraries(main
        PUBLIC ${OpenCV_LIBS}
        fit)

target_link_libraries(Armors_fit
        PUBLIC ${OpenCV_LIBS}
        fit)
