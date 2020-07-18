cmake_minimum_required(VERSION 3.13)
# Project information
project(Control
  VERSION 1.0.0
  DESCRIPTION "Control laws."
  LANGUAGES CXX
)
# Add the library target
add_library(control INTERFACE)
# Setup include directories 
target_include_directories(control INTERFACE include/)
# Example and unit testing if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	# Add the example target
	add_executable(control_example examples/control_example.cc)
	# Add the includes
	target_include_directories(control_example PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
	# Link libraries to the test target
	target_link_libraries(control_example
		PRIVATE 
			control
	)
# 	# Fetch google test
# 	include(FetchContent)
# 	FetchContent_Declare(
# 		googletest
# 		GIT_REPOSITORY 	https://github.com/google/googletest.git
# 		GIT_TAG					release-1.10.0
# 	)
# 	FetchContent_MakeAvailable(googletest)
# 	enable_testing()
# 	include(GoogleTest)
# 	# Add the test target
# 	add_executable(types_test tests/types_test.cc)
# 	# Add the includes
# 	target_include_directories(types_test PUBLIC 
# 		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 		$<INSTALL_INTERFACE:include>
# 	)
# 	# Link libraries to the test target
# 	target_link_libraries(types_test
# 		PRIVATE 
# 			types
# 			gtest_main 
# 			gtest
# 			gmock
# 	)
# 	# Discover unit tests
# 	gtest_discover_tests(types_test)
endif()
