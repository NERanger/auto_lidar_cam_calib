add_library(kitti_loader STATIC
	kitti_loader/KittiLoader.cc
)
target_compile_features(kitti_loader PUBLIC cxx_std_17)
target_link_libraries(kitti_loader PUBLIC Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_include_directories(kitti_loader PUBLIC 
	${CMAKE_CURRENT_LIST_DIR}
)

add_library(mem2disk STATIC
	mem2disk/Mem2Disk.cc
)
target_compile_features(mem2disk PUBLIC cxx_std_17)
target_include_directories(mem2disk PUBLIC 
	${CMAKE_CURRENT_LIST_DIR}
)

add_library(livox_loader STATIC 
	livox_loader/LivoxLoader.cc)
target_compile_features(livox_loader PUBLIC cxx_std_17)
target_link_libraries(livox_loader PUBLIC Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} glog::glog)
target_include_directories(livox_loader PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(external_deps INTERFACE)

target_link_libraries(external_deps INTERFACE
	kitti_loader
	livox_loader
	mem2disk
)