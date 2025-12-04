cmake_minimum_required(VERSION 3.10)
project(dobot_fk_ik_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/scripts/fk_ik_sympy.py
  ${PROJECT_NAME}/scripts/fk_to_rviz.py
  ${PROJECT_NAME}/scripts/ik_to_rviz.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Allow ros2 to discover the package
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
