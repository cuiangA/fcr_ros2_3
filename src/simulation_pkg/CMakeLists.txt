# simulation_pkg CMakeLists.txt
# 协同控制仿真包

cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/coop_sim.py
  scripts/servo_chain_mock_source.py
  scripts/servo_chain_acceptance.py
  scripts/servo_bag_eval_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
