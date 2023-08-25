# pcl
#find_package( PCL REQUIRED COMPONENT common io)
find_package( PCL 1.8 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)  #指定pcl1.8路径
include_directories("/usr/include/eigen3")
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( pcd2ply src/pcd2ply.cpp )
target_link_libraries( pcd2ply ${OpenCV_LIBS} ${PCL_LIBRARIES} )

