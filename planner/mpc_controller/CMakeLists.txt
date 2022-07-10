cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

message(status "IPOPT_CFLAGS: " ${IPOPT_CFLAGS})
message(status "IPOPT_CFLAGS_OTHER: " ${IPOPT_CFLAGS_OTHER})
set(CMAKE_CXX_FLAGS "-DHAVE_CSTDDEF -DHAVE_MPI_INITIALIZED")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp

        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mpc_controller

        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

add_library(mpc_controller
        src/MPC.cpp
        include/MPC.hpp)
target_link_libraries(mpc_controller
        ${catkin_LIBRARIES}
        )

include_directories(
        include
        /usr/local/include
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)
link_directories(/usr/local/lib)

add_definitions(${EIGEN3_DEFINITIONS})



add_executable(solver app/example.cpp)
target_link_libraries(solver ipopt)

add_executable(qp_test app/qp_test.cpp)
target_link_libraries(qp_test libqpOASES.so)

add_executable(mpc_test app/mpc_control_test.cpp
                        src/uniform_bspline.cpp
                        src/MPC.cpp)
target_link_libraries(mpc_test
        libqpOASES.so
        ${catkin_LIBRARIES}
        )
