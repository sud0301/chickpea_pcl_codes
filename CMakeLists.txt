cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract_indices)

find_package(PCL 1.7 REQUIRED)
find_package (OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (segment_light segment_light.cpp)

target_link_libraries (segment_light libv4rSegmenterLight.so libv4rTomGine.so)
#target_link_libraries (segment_light usr/local/libboost_system.so.1.46.1)
target_link_libraries (segment_light ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable (pcl_visual pcl_visual.cpp)
add_executable (normal_based_clustering normal_based_clustering.cpp)
add_executable (example_supervoxels example_supervoxels.cpp)
add_executable (ocv_test ocv_test.cpp)
add_executable (ppm_pgm_to_pcd ppm_pgm_to_pcd.cpp)
add_executable (bilateral_filtering bilateral_filtering.cpp)
add_executable (example_remove_nan_from_point_cloud example_remove_nan_from_point_cloud.cpp )
add_executable (resampling resampling.cpp)
#add_executable (bilateral_upsampling bilateral_upsampling.cpp)
add_executable  (align_time_rgb align_time_rgb.cpp)
add_executable  (fetch_transform fetch_transform.cpp)
add_executable (interactive_icp interactive_icp.cpp)
add_executable (supervoxel_clustering supervoxel_clustering.cpp)

target_link_libraries (ppm_pgm_to_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS}) 
target_link_libraries (ocv_test ${OpenCV_LIBS})
target_link_libraries (example_supervoxels ${PCL_LIBRARIES})
target_link_libraries (normal_based_clustering ${PCL_LIBRARIES})
target_link_libraries (pcl_visual ${PCL_LIBRARIES})
target_link_libraries (bilateral_filtering ${PCL_LIBRARIES})
target_link_libraries (example_remove_nan_from_point_cloud ${PCL_LIBRARIES} )
target_link_libraries (resampling ${PCL_LIBRARIES})
target_link_libraries (align_time_rgb ${PCL_LIBRARIES})
target_link_libraries (fetch_transform ${PCL_LIBRARIES})
target_link_libraries (interactive_icp ${PCL_LIBRARIES})
target_link_libraries (supervoxel_clustering ${PCL_LIBRARIES})
#target_link_libraries (bilateral_upsampling ${PCL_LIBRARIES})
#target_link_libraries (b_filtering ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} )
#TARGET_LINK_LIBRARIES("/usr/local/lib/libv4rSegmenterLight.so")



