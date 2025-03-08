cmake_minimum_required(VERSION 3.8)
project(c2hi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install launch and config files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/clip_vis_rec_server.py  
  scripts/gesture_rec_server.py
  scripts/semantic_fusion_node.py
  scripts/experiments/cmd_exp_mgr.py
  scripts/experiments/role_exp_mgr.py
  scripts/experiments/scene_exp_mgr.py
  scripts/experiments/record_scene_results_node.py
  scripts/experiments/record_hierarchical_cmd_results_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
