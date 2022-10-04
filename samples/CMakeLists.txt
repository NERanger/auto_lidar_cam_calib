add_executable(sample sample.cc)
target_link_libraries(sample kitti_loader auto_cali)

add_executable(sample_livox sample_livox.cc)
target_link_libraries(sample_livox livox_loader auto_cali)