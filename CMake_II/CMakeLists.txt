cmake_minimum_required(VERSION 3.10)
project(Test)
add_compile_options(-std=gnu++17)
find_package(OpenCV)
# # # # 以下内容禁止修改 # # # #
option(BUILD_A "build modules a" ON)
option(BUILD_B "build modules b" OFF)
option(BUILD_TESTS "build tests" OFF)

if(BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(modules)
add_subdirectory(common)
# 小提示: module2 需要用到 pthread，即 target_link_libraries(xxx pthread)
# # # # 以上内容禁止修改 # # # #
if (BUILD_A)
    add_definitions(-DWITH_A)
endif ()
if (BUILD_B)
    add_definitions(-DWITH_B)
endif ()

add_executable(server server.cpp)
add_executable(client client.cpp)

target_include_directories(server PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(client PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(client opcua_cs)
target_link_libraries(server PUBLIC
        module1
        rmath
        singleton
        opcua_cs
        ${OpenCV_LIBS}
        )
add_test(NAME mytest COMMAND Test)