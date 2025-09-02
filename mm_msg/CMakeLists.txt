cmake_minimum_required(VERSION 3.10)  # 设置最低支持的 CMake 版本
project(mm_msg)                      # 定义项目名称

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

# 添加消息文件
add_message_files(
  FILES
  TargetTrajectories.msg
  SolverInput.msg
  State.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 定义 catkin 包
catkin_package(
  CATKIN_DEPENDS message_runtime
)
