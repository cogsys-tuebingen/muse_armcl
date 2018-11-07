cmake_minimum_required(VERSION 2.8.3)
project(muse_armcl)


find_package(catkin REQUIRED COMPONENTS
  muse_smc
  cslibs_plugins
  cslibs_plugins_data
)

catkin_package(
  INCLUDE_DIRS    include
  CATKIN_DEPENDS  muse_smc cslibs_plugins cslibs_plugins_data
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
  src/node/muse_armcl_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
