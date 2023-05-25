cmake_minimum_required(VERSION 3.0.2)
project(mpc_towr)

find_package(ifopt 2.0.1 REQUIRED)
find_package(lcm REQUIRED)


add_executable(trunk_mpc trunk_mpc.cpp)
target_link_libraries(trunk_mpc PUBLIC
    towr
    ifopt::ifopt_ipopt
    lcm
)