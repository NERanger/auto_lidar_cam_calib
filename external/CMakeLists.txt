add_library(kitti_loader STATIC
	kitti_loader/KittiLoader.cc
)
target_link_libraries(kitti_loader PUBLIC Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_include_directories(kitti_loader PUBLIC 
	${CMAKE_CURRENT_LIST_DIR}
)