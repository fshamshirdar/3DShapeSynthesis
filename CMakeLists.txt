cmake_minimum_required(VERSION 2.8.12)

project(smf_view)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(glui REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(smf_view
	src/main.cpp
	src/smf_parser.cpp
	src/face.cpp
	src/vertex.cpp
	src/region.cpp
	src/winged_edge.cpp
)
target_link_libraries(smf_view glui::glui_static ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${EIGEN_LIBRARIES})
