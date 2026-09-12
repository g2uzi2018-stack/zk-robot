#include "motion/kinematics/urdf_chain.hpp"

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace robot::motion
{

    UrdfChain loadUrdfChain(
        const std::string &urdf_path,
        const std::string &base_link,
        const std::string &tip_link,
        const std::size_t expected_joint_count)
    {
        // 1. 检查调用者提供的参数。
        if (urdf_path.empty() || base_link.empty() || tip_link.empty())
        {
            throw std::runtime_error("URDF path, base link and tip link must not be empty");
        }

        if (base_link == tip_link || expected_joint_count == 0)
        {
            throw std::runtime_error("Provide distinct base/tip links and a positive joint count");
        }

        // 2. 读取 URDF，得到 urdfdom 提供的机器人内存模型。
        const auto robot = urdf::parseURDFFile(urdf_path);

        if (!robot)
        {
            throw std::runtime_error("Failed to parse URDF file: " + urdf_path);
        }

        if (!robot->getLink(base_link))
        {
            throw std::runtime_error("Base link does not exist: " + base_link);
        }

        auto current_link = robot->getLink(tip_link);

        if (!current_link)
        {
            throw std::runtime_error(
                "Tip link does not exist: " + tip_link);
        }

        // 准备我们自己的返回数据。
        UrdfChain result{base_link, tip_link, {}, {}};

        // 防止沿父关节回溯时重复访问同一个连杆。
        std::unordered_set<std::string> visited_links;

        // 3. 从末端往前找，直到到达指定的基准连杆。
        while (current_link->name != base_link)
        {
            if (!visited_links.insert(current_link->name).second)
            {
                throw std::runtime_error(
                    "Cycle detected while extracting the URDF chain");
            }

            // 这个连杆通过哪个关节连接到它的父连杆？
            const auto source_joint = current_link->parent_joint;

            if (!source_joint)
            {
                throw std::runtime_error(
                    "Base link is not an ancestor of tip link");
            }

            // 第一版不处理 mimic 联动关节。
            if (source_joint->mimic)
            {
                throw std::runtime_error(
                    "Mimic joint is not supported: " + source_joint->name);
            }

            // 将 urdfdom 的一个关节，转换成我们自己的关节数据。
            UrdfChainJoint joint;

            joint.name = source_joint->name;
            joint.parent_link = source_joint->parent_link_name;
            joint.child_link = source_joint->child_link_name;

            switch (source_joint->type)
            {
            case urdf::Joint::FIXED:
                joint.type = UrdfJointType::Fixed;
                break;

            case urdf::Joint::REVOLUTE:
                joint.type = UrdfJointType::Revolute;
                break;

            case urdf::Joint::CONTINUOUS:
                joint.type = UrdfJointType::Continuous;
                break;

            case urdf::Joint::PRISMATIC:
                joint.type = UrdfJointType::Prismatic;
                break;

            default:
                throw std::runtime_error(
                    "Unsupported joint type: " + joint.name);
            }

            // 读取的是 joint/origin，不是 visual/origin。
            const auto &origin =
                source_joint->parent_to_joint_origin_transform;

            joint.origin.position = Eigen::Vector3d{
                origin.position.x,
                origin.position.y,
                origin.position.z};

            // urdfdom 已经把 URDF 中的姿态解析成四元数。
            // Eigen 这个构造函数的参数顺序是 w、x、y、z。
            joint.origin.orientation = Eigen::Quaterniond{
                origin.rotation.w,
                origin.rotation.x,
                origin.rotation.y,
                origin.rotation.z};

            try
            {
                joint.origin = normalizedPose(joint.origin);
            }
            catch (const std::invalid_argument &error)
            {
                throw std::runtime_error(
                    "Invalid origin for joint " + joint.name +
                    ": " + error.what());
            }

            // 活动关节需要运动轴；固定关节不使用 axis。
            if (joint.type != UrdfJointType::Fixed)
            {
                joint.axis = Eigen::Vector3d{
                    source_joint->axis.x,
                    source_joint->axis.y,
                    source_joint->axis.z};

                const double axis_norm = joint.axis.stableNorm();

                if (!joint.axis.allFinite() ||
                    !std::isfinite(axis_norm) ||
                    axis_norm <= 1e-12)
                {
                    throw std::runtime_error(
                        "Invalid axis for joint: " + joint.name);
                }

                // 只把轴变成单位向量。
                // 它仍然表达在关节局部坐标系中，没有转换到父连杆坐标系。
                joint.axis /= axis_norm;
            }

            result.joints.push_back(std::move(joint));

            // 继续往前找：当前关节的父连杆，成为下一轮的当前连杆。
            current_link = robot->getLink(source_joint->parent_link_name);

            if (!current_link)
            {
                throw std::runtime_error(
                    "Parent link does not exist: " +
                    source_joint->parent_link_name);
            }
        }

        // 4. 刚才是 tip -> base 收集的，现在反转成 base -> tip。
        std::reverse(result.joints.begin(), result.joints.end());

        // 5. 按真正的运动链顺序，分配活动关节的 q 下标。
        for (auto &joint : result.joints)
        {
            if (joint.type == UrdfJointType::Fixed)
            {
                continue;
            }

            joint.q_index = result.joint_names.size();
            result.joint_names.push_back(joint.name);
        }

        // 6. 检查活动关节数量是否符合调用者的预期。
        if (result.joint_names.size() != expected_joint_count)
        {
            throw std::runtime_error(
                "Active joint count mismatch: expected " +
                std::to_string(expected_joint_count) +
                ", got " +
                std::to_string(result.joint_names.size()));
        }

        return result;
    }

} // namespace robot::motion