find_package(rostest REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

catkin_add_gtest(test_${PROJECT_NAME}
  homework4_test.cpp
  ../src/ObjectEkf.cpp
)
target_include_directories(test_${PROJECT_NAME} PUBLIC
  ${catkin_INCLUDE_DIRS}
  ../src
)
add_dependencies(test_${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_${PROJECT_NAME}
  ${catkin_LIBRARIES}
  jsoncpp
)
