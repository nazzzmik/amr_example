if(EXISTS "C:/Users/Dell I7/Downloads/amr_example_small_no_ROS/amr_example_small_no_ROS/cmake-build-debug/RunAmrTests[1]_tests.cmake")
  include("C:/Users/Dell I7/Downloads/amr_example_small_no_ROS/amr_example_small_no_ROS/cmake-build-debug/RunAmrTests[1]_tests.cmake")
else()
  add_test(RunAmrTests_NOT_BUILT RunAmrTests_NOT_BUILT)
endif()
