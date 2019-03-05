cmake_minimum_required(VERSION 2.8.3)
project(muse_armcl)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    muse_smc
    cslibs_plugins
    cslibs_plugins_data
    cslibs_mesh_map
    cslibs_indexed_storage
    cslibs_kdl
    cslibs_kdl_data
    cslibs_kdl_msgs
    cslibs_kdl_conversion
    cslibs_utility
    rosbag
    )

find_package(jaco2_contact_msgs QUIET)
find_package(orocos_kdl REQUIRED)
find_package(NLOPT REQUIRED)

catkin_package(
    INCLUDE_DIRS   include
    CATKIN_DEPENDS muse_smc cslibs_plugins cslibs_plugins_data
    cslibs_mesh_map cslibs_indexed_storage cslibs_kdl cslibs_utility rosbag cslibs_kdl_msgs cslibs_kdl_data cslibs_kdl_conversion
    DEPENDS orocos_kdl NLOPT
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}_lib SHARED
    # contains all plugins...
    src/state_space/mesh_map_loader.cpp
    src/state_space/mesh_map_loader_offline.cpp
    src/resampling/kld.cpp
    src/resampling/sir.cpp
    src/resampling/residual.cpp
    src/resampling/stratified.cpp
    src/resampling/systematic.cpp
    src/resampling/wheel.cpp
    src/resampling/kld_random.cpp
    src/sampling/normal.cpp
    src/sampling/uniform_all_links.cpp
    src/sampling/uniform_per_link.cpp
    src/scheduling/cfs.cpp
    src/scheduling/rate.cpp
    src/scheduling/dummy.cpp
    src/scheduling/integrate_all.cpp
    src/density/dominants.cpp
    src/density/means.cpp
    src/density/weighted_means.cpp
    src/density/nearest_neighbor_density.cpp
    src/density/contact_point_histogram.cpp
    src/density/contact_point_histogram_min.cpp
    src/prediction/random_walk.cpp
    src/update/joint_state_provider.cpp
    src/update/normalized_update_model.cpp
    src/update/normalized_cone_update_model.cpp
    src/state_space/transform_graph.cpp
    )
target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${NLOPT_LIBRARIES}
    )


add_executable(${PROJECT_NAME}_node
    src/node/muse_armcl_node.cpp
    src/state_space/state_publisher.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_lib
    ${orocos_kdl_LIBRARIES}
    ${NLOPT_LIBRARIES}
    )

if(${jaco2_contact_msgs_FOUND})

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${orocos_kdl_INCLUDE_DIRS}
        ${NLOPT_INCLUDE_DIRS}
        ${jaco2_contact_msgs_INCLUDE_DIRS}
        )

    add_executable(${PROJECT_NAME}_test_bag_node
        src/node/muse_armcl_test_bag_node.cpp
        )

    target_link_libraries(${PROJECT_NAME}_test_bag_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_lib
        ${orocos_kdl_LIBRARIES}
        ${NLOPT_LIBRARIES}
        ${jaco2_contact_msgs_LIBRARIES}
        )

    add_executable(${PROJECT_NAME}_test_tfg_node
        src/node/muse_armcl_test_tfg_node
        )
    target_link_libraries(${PROJECT_NAME}_test_tfg_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_lib
        ${orocos_kdl_LIBRARIES}
        ${NLOPT_LIBRARIES}
        ${jaco2_contact_msgs_LIBRARIES}
        )


    add_executable(${PROJECT_NAME}_offline_node
        src/node/muse_armcl_offline_node.cpp
        src/state_space/state_publisher.cpp
        src/state_space/state_publisher_offline.cpp
        )

    add_dependencies(${PROJECT_NAME}_offline_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    target_link_libraries(${PROJECT_NAME}_offline_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_lib
        ${orocos_kdl_LIBRARIES}
        ${NLOPT_LIBRARIES}
        ${jaco2_contact_msgs_LIBRARIES}
        )

    add_executable(${PROJECT_NAME}_labeled_contact_points_change_frames
        src/node/labeled_contact_points_change_frames.cpp
        )

    add_dependencies(${PROJECT_NAME}_labeled_contact_points_change_frames ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    target_link_libraries(${PROJECT_NAME}_labeled_contact_points_change_frames
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_lib
        ${orocos_kdl_LIBRARIES}
        ${NLOPT_LIBRARIES}
        ${jaco2_contact_msgs_LIBRARIES}
        )

    add_executable(${PROJECT_NAME}_bag_file_data_publisher
        src/node/muse_armcl_bag_file_data_publisher.cpp
        )

    add_dependencies(${PROJECT_NAME}_bag_file_data_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    target_link_libraries(${PROJECT_NAME}_bag_file_data_publisher
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_lib
        ${orocos_kdl_LIBRARIES}
        ${NLOPT_LIBRARIES}
        ${jaco2_contact_msgs_LIBRARIES}
        )

endif()

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
