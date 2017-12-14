cmake_minimum_required(VERSION 2.8.12)

project(smf_view)

set(CMAKE_BUILD_TYPE Debug)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(glui REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
include_directories(src/ ${EIGEN3_INCLUDE_DIR})

add_executable(smf_view
	src/main.cpp
	src/smf_parser.cpp
	src/face.cpp
	src/vertex.cpp
	src/region.cpp
	src/part.cpp
	src/part_intersection.cpp
	src/data.cpp
	src/utils.cpp
	src/mix_match.cpp
	src/mixers/simple_box.cpp
	src/mixers/intersection_box.cpp
	src/mixers/missing_part.cpp
	src/mixers/missing_intersection_part.cpp
	src/mixers/knn_weighted_interpolation.cpp
	src/mixers/rigid_transform.cpp
	src/control_points.cpp
	src/control_points/eight_points.cpp
	src/control_points/closest_connecting_points.cpp
	src/control_points/hull_grid_points.cpp
	src/control_points/box_intersection_points.cpp
	src/control_points/box_intersection_target_points.cpp
	src/control_points/missing_connecting_points.cpp
)
target_link_libraries(smf_view glui::glui_static ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${EIGEN_LIBRARIES})
