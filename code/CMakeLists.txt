cmake_minimum_required(VERSION 2.8)

project( objectDetector )

add_executable( objectDetector main.cpp )

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS program_options REQUIRED )


target_link_libraries( objectDetector ${OpenCV_LIBS} ${Boost_LIBRARIES})
