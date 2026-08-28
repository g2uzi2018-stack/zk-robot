#pragma once

#include <spdlog/spdlog.h>

namespace robot::common
{

    // 获取项目统一 logger。
    //
    // 当前实现基于 spdlog 默认 logger。
    // 后续如果需要：
    // - 文件输出
    // - 日志轮转
    // - 不同 sink
    // - ROS2 logging
    //
    // 只需要修改这里，不影响业务代码。
    inline std::shared_ptr<spdlog::logger> logger()
    {
        return spdlog::default_logger();
    }

}