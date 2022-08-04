add_library(
	auto_cali STATIC
		Calibrator.cc
		Core.cc
		Types.cc
		Constants.cc
		Misc.cc
		Rand.cc
		StopWatch.cc
)
target_compile_features(auto_cali PUBLIC cxx_std_17)
target_link_libraries(auto_cali PUBLIC Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} glog::glog mem2disk)
target_include_directories(auto_cali PUBLIC ${CMAKE_CURRENT_LIST_DIR})