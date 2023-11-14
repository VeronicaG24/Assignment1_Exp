cmake_minimum_required(VERSION 2.8.3)
project(rosbot_descr_real)

find_package(catkin REQUIRED
	rospy
)

catkin_python_setup()

catkin_package(
)

catkin_install_python(PROGRAMS
  scripts/rosbot_descr_real/serial_bridge.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbot_descr_real
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY rviz
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
