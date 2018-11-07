cmake_minimum_required(VERSION 2.8.3)
project(muse_armcl)


find_package(catkin REQUIRED COMPONENTS
  muse_smc
  cslibs_plugins
  cslibs_plugins_data
  cslibs_mesh_map
)

catkin_package(
  INCLUDE_DIRS    include
  CATKIN_DEPENDS  muse_smc cslibs_plugins cslibs_plugins_data cslibs_mesh_map
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#add_library(${PROJECT_NAME}_lib
#  # put plugins here
#)
#target_link_libraries(${PROJECT_NAME}_lib
#  ${catkin_LIBRARIES}
#)


add_executable(${PROJECT_NAME}_node
  src/node/muse_armcl_node.cpp
  src/state_space/state_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}_lib
)


install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(TARGETS ${PROJECT_NAME}_lib
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
