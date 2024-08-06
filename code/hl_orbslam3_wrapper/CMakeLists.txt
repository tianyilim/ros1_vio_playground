cmake_minimum_required(VERSION 3.0)
project(hl_orbslam3_wrapper)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    message_filters
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    message_filters
    LIBRARIES {PROJECT_NAME}
)

catkin_python_setup()

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY cfg/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
install(DIRECTORY viz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/viz)

catkin_install_python(PROGRAMS
    nodes/imu_downsampling_node.py
    nodes/dummy_imu_pub_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
