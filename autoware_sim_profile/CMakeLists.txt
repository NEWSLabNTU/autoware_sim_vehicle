cmake_minimum_required(VERSION 3.14)
project(autoware_sim_profile)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# `ros2 run autoware_sim_profile setup_autoware_data` -- mirrors Autoware's packaged data into a
# writable data_path (see the script's header).
install(PROGRAMS scripts/setup_autoware_data DESTINATION lib/${PROJECT_NAME})
# The planning common parameter file for carla_simulator's max_velocity, run by a
# $(command) substitution while the launch resolves (see the script's header).
install(PROGRAMS scripts/planning_common_param DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/clock_guard DESTINATION lib/${PROJECT_NAME})
# Low obstacles on the lane against the map (scensim issue 180; see the script's header).
install(PROGRAMS scripts/lane_height_check DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/check_launch_args DESTINATION lib/${PROJECT_NAME})
# The compare-map filter's parameter file for a map: dynamic loading for a divided map, static
# otherwise (scensim issue 072), run by a $(command) substitution (see the script's header).
install(PROGRAMS scripts/pointcloud_map_filter_param DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
