cmake_minimum_required(VERSION 3.8)
project(grupo08_kuka_iisy11_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
endif()
install(PROGRAMS
  cinematicas/cinematica_directa.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  cinematicas/cinematica_inversa.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
