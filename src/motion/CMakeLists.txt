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


# 找到 pkg-config 工具，用它查询已安装的 urdfdom。
find_package(PkgConfig REQUIRED)

# 查找 urdfdom，并把它的头文件路径、库文件等信息
# 整理成 PkgConfig::URDFDOM 这个 CMake 目标。
pkg_check_modules(URDFDOM REQUIRED IMPORTED_TARGET GLOBAL urdfdom)

# 把我们写的 URDF 加载代码编译成独立的静态库。
add_library(robot_urdf_chain STATIC
    kinematics/urdf_chain.cpp
)

# 写法	             编译 robot_urdf_chain 自己时	 编译使用它的测试程序时
# PUBLIC 某个依赖	 使用这个依赖提供的编译要求	       也传给测试程序
# PRIVATE 某个依赖	 使用这个依赖提供的编译要求	       不通过这条依赖继续传给测试程序

# 声明这个静态库需要哪些依赖。
target_link_libraries(robot_urdf_chain
    PUBLIC
        robot_motion
    PRIVATE
        PkgConfig::URDFDOM
)