cmake_minimum_required(VERSION 3.8)
project(motiondock_user_tasks)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

file(GLOB USER_TASK_SOURCES CONFIGURE_DEPENDS "tasks/*.cpp" "tasks/*.cc" "tasks/*.cxx")
foreach(task_source IN LISTS USER_TASK_SOURCES)
  get_filename_component(task_stem "${task_source}" NAME_WE)
  set(task_target "task_${task_stem}")
  add_executable("${task_target}" "${task_source}")
  ament_target_dependencies("${task_target}" rclcpp)
  install(TARGETS "${task_target}" DESTINATION "lib/${PROJECT_NAME}")
endforeach()

ament_package()
