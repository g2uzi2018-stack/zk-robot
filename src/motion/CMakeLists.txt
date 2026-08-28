# motion 当前由纯头文件算法组成。INTERFACE target 统一向所有调用者传播
# src include 路径和 Eigen 依赖，避免每个 Executor 或应用重复配置。
add_library(robot_motion INTERFACE)

target_include_directories(robot_motion INTERFACE
    ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(robot_motion INTERFACE
    Eigen3::Eigen
)

target_compile_features(robot_motion INTERFACE
    cxx_std_17
)
