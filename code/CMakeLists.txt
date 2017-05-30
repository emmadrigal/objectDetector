cmake_minimum_required(VERSION 2.8)

project( objectDetector )

add_executable( objectDetector main.cpp )

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS program_options REQUIRED )

set_property(TARGET objectDetector PROPERTY CXX_STANDARD 11)

target_link_libraries( objectDetector ${OpenCV_LIBS} ${Boost_LIBRARIES})
