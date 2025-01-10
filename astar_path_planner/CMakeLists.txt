cmake_minimum_required(VERSION 3.0.2)
project(astar_path_planner)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        visualization_msgs
        geometry_msgs
        nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package()

set(osqp_INCLUDE_DIRS /usr/local/include/osqp)
set(OSQP_EIGEN_INCLUDE_DIR /usr/local/include/OsqpEigen)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${osqp_INCLUDE_DIRS}
        ${OSQP_EIGEN_INCLUDE_DIR}
)

add_executable(obstacle_generator src/obstacle_generator.cpp)
target_link_libraries(obstacle_generator ${catkin_LIBRARIES})


add_executable(astar_planner src/astar_planner.cpp)
target_link_libraries(astar_planner ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(Trajectory src/Trajectory.cpp)
target_link_libraries(Trajectory ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} OsqpEigen::OsqpEigen)
