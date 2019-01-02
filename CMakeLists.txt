project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(core_sources src/MPC.cpp src/geometry.cpp src/Localizer.cpp src/client.cpp src/model.cpp src/PID.cpp)

# This is where I installed Ipopt
include_directories($ENV{HOME}/libs/include)
link_directories($ENV{HOME}/libs/lib)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/opt/openssl/lib)

    file(GLOB LIBUV_DIR /usr/local/Cellar/libuv/1*/lib)
    link_directories(${LIBUV_DIR})

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc src/main.cpp ${core_sources})
target_link_libraries(mpc ipopt z ssl uv uWS)

add_executable(play src/play.cpp ${core_sources})
target_link_libraries(play ipopt z ssl uv uWS)

add_executable(example src/example.cpp src/plotsim.cpp ${core_sources})
target_link_libraries(example ipopt z ssl uv uWS python)

