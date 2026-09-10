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

# 初始化阶段的可选 URDF 数据加载层；不让 XML 依赖进入原有纯数学目标。
option(BUILD_URDF_CHAIN_LOADER "Build the offline URDF chain loader and its tests" OFF)
if(BUILD_URDF_CHAIN_LOADER)
    find_package(LibXml2 REQUIRED)
    add_library(robot_urdf_chain STATIC
        kinematics/urdf_chain.cpp
    )
    target_link_libraries(robot_urdf_chain PUBLIC robot_motion PRIVATE LibXml2::LibXml2)
endif()
