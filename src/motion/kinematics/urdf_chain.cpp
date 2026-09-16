#include "motion/kinematics/urdf_chain.hpp"

#include <urdf_parser/urdf_parser.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace robot::motion
{
    namespace
    {
        using XmlTree = boost::property_tree::ptree;

        /**
         * 仅由 loadUrdfChain() 调用，读取一个关节明确声明的位置区间。
         * source：urdfdom 解析出的关节，提供已经转换为 double 的上下界。
         * joint_xml：同一份文件快照中该 <joint> 的原始 XML 子树。
         *
         * 为什么还需要 XML？部分 urdfdom/URDF 版本会给缺失端点补默认值，
         * 只看 source.limits 的数值，无法区分“明确写了 0”和“没有写”。
         * 本函数用 XML 判断字段存在性，不重新计算几何或重新转换上下界数值。
         *
         * Revolute / Prismatic：
         *   两端都明确提供：构造 JointPositionBounds，检查有限性和顺序。
         *   两端都未提供：返回 nullopt，表示模型约束缺失，不表示无限制。
         *   只提供一端：拒绝半个区间，不猜测另一端，也不自动用配置覆盖。
         * Fixed / Continuous：返回 nullopt，不把无关的上下界用作位置限制。
         *
         * 前提：整份 URDF 已成功通过当前版本 urdfdom 的解析。
         * 本函数不能让 urdfdom 本身拒绝的文件变成合法模型。
         * 返回值交给 joint.position_bounds；失败抛出带关节名的 runtime_error。
         */
        std::optional<JointPositionBounds> readDeclaredPositionBounds(
            const urdf::Joint &source,
            const XmlTree &joint_xml)
        {
            if (source.type == urdf::Joint::FIXED ||
                source.type == urdf::Joint::CONTINUOUS)
            {
                return std::nullopt;
            }
            if (source.type != urdf::Joint::REVOLUTE &&
                source.type != urdf::Joint::PRISMATIC)
            {
                throw std::runtime_error("Unsupported joint type: " + source.name);
            }

            if (joint_xml.count("limit") > 1)
            {
                throw std::runtime_error("Repeated <limit> at joint: " + source.name);
            }
            const auto limit_xml = joint_xml.get_child_optional("limit");
            if (!limit_xml)
            {
                return std::nullopt;
            }

            const auto attributes = limit_xml->get_child_optional("<xmlattr>");
            if (attributes && (attributes->count("lower") > 1 ||
                               attributes->count("upper") > 1))
            {
                throw std::runtime_error("Repeated position bound at joint: " + source.name);
            }

            const auto lower_text = limit_xml->get_optional<std::string>("<xmlattr>.lower");
            const auto upper_text = limit_xml->get_optional<std::string>("<xmlattr>.upper");
            if (!lower_text && !upper_text)
            {
                return std::nullopt;
            }
            if (!lower_text || !upper_text)
            {
                throw std::runtime_error(
                    "Both lower and upper must be provided together at joint: " + source.name);
            }
            if (!source.limits)
            {
                throw std::runtime_error("Parsed limits are missing at joint: " + source.name);
            }

            try
            {
                return JointPositionBounds{source.limits->lower, source.limits->upper};
            }
            catch (const std::invalid_argument &error)
            {
                throw std::runtime_error(
                    "Invalid position bounds at joint " + source.name + ": " + error.what());
            }
        }
    } // namespace

    /**
     * 上游：初始化代码传入文件路径、base/tip 名称与预期活动关节数。
     * 下游：返回包含几何、q 下标和模型位置范围的原始 UrdfChain。
     *
     * 文件只读取一次：XML 字段检查与 urdfdom 模型解析使用同一个文本快照。
     * 不修改 XML、不读取电机配置、不访问硬件、不合并任务约束、不执行 IK。
     * 本实现需要上一轮含 position_bounds 的 urdf_chain.hpp。
     * Boost.PropertyTree 只用于初始化时检查原始 XML 字段，使用头文件即可。
     *
     * 坐标约定：origin/axis 不变；限位与对应 q 使用模型零位和正方向。
     * 旋转位置单位 rad，移动位置单位 m；固定关节不占用 q。
     *
     * 成功加载不等于整组 IK 约束完整：若有界关节缺少 position_bounds，
     * 后续约束初始化必须由明确配置补齐，或拒绝开始带约束求解。
     * 文件/模型错误报告 runtime_error；失败时不返回部分运动链。
     */
    UrdfChain loadUrdfChain(
        const std::string &urdf_path,
        const std::string &base_link,
        const std::string &tip_link,
        const std::size_t expected_joint_count)
    {
        if (urdf_path.empty() || base_link.empty() || tip_link.empty())
        {
            throw std::runtime_error("URDF path, base link and tip link must not be empty");
        }
        if (base_link == tip_link || expected_joint_count == 0)
        {
            throw std::runtime_error("Provide distinct base/tip links and a positive joint count");
        }

        /** 一次读入原始文本，避免先后读文件时得到两个不同版本。 */
        std::ifstream input{urdf_path, std::ios::binary};
        if (!input)
        {
            throw std::runtime_error("Cannot open URDF file: " + urdf_path);
        }
        std::ostringstream buffer;
        buffer << input.rdbuf();
        if (input.bad() || buffer.fail())
        {
            throw std::runtime_error("Failed to read URDF file: " + urdf_path);
        }
        const std::string xml_text = buffer.str();
        if (xml_text.empty() || xml_text.find('\0') != std::string::npos)
        {
            throw std::runtime_error("URDF is empty or contains a NUL byte: " + urdf_path);
        }

        XmlTree document;
        std::istringstream xml_input{xml_text};
        try
        {
            boost::property_tree::read_xml(
                xml_input, document, boost::property_tree::xml_parser::no_comments);
        }
        catch (const boost::property_tree::xml_parser_error &error)
        {
            throw std::runtime_error("Invalid XML in " + urdf_path + ": " + error.what());
        }
        if (document.count("robot") != 1)
        {
            throw std::runtime_error("Expected exactly one <robot> element: " + urdf_path);
        }

        /** 改为解析同一份文本；几何解析仍由 urdfdom 完成。 */
        const auto robot = urdf::parseURDF(xml_text);
        if (!robot)
        {
            throw std::runtime_error("Failed to parse URDF model: " + urdf_path);
        }

        /**
         * 只索引 <robot> 的直接 <joint> 子元素，不误读 transmission 等扩展。
         * value 是只读 XML 节点的地址；document 在本函数结束前一直存活，
         * 建立索引后不再修改 document，返回的运动链也不保存这些地址。
         */
        std::unordered_map<std::string, const XmlTree *> joint_xml_by_name;
        for (const auto &element : document.get_child("robot"))
        {
            if (element.first != "joint")
            {
                continue;
            }
            const auto name = element.second.get_optional<std::string>("<xmlattr>.name");
            if (!name || name->empty() ||
                !joint_xml_by_name.emplace(*name, &element.second).second)
            {
                throw std::runtime_error("Empty or repeated joint name in XML");
            }
        }

        if (!robot->getLink(base_link))
        {
            throw std::runtime_error("Base link does not exist: " + base_link);
        }
        auto current_link = robot->getLink(tip_link);
        if (!current_link)
        {
            throw std::runtime_error("Tip link does not exist: " + tip_link);
        }

        UrdfChain result{base_link, tip_link, {}, {}};
        std::unordered_set<std::string> visited_links;
        while (current_link->name != base_link)
        {
            if (!visited_links.insert(current_link->name).second)
            {
                throw std::runtime_error("Cycle detected while extracting the URDF chain");
            }
            const auto source_joint = current_link->parent_joint;
            if (!source_joint)
            {
                throw std::runtime_error("Base link is not an ancestor of tip link");
            }
            if (source_joint->mimic)
            {
                throw std::runtime_error("Mimic joint is not supported: " + source_joint->name);
            }

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
                throw std::runtime_error("Unsupported joint type: " + joint.name);
            }

            /** joint/origin，不是 visual/origin；四元数构造参数为 w、x、y、z。 */
            const auto &origin = source_joint->parent_to_joint_origin_transform;
            joint.origin.position = Eigen::Vector3d{
                origin.position.x, origin.position.y, origin.position.z};
            joint.origin.orientation = Eigen::Quaterniond{
                origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z};
            try
            {
                joint.origin = normalizedPose(joint.origin);
            }
            catch (const std::invalid_argument &error)
            {
                throw std::runtime_error(
                    "Invalid origin for joint " + joint.name + ": " + error.what());
            }
            if (joint.type != UrdfJointType::Fixed)
            {
                joint.axis = Eigen::Vector3d{
                    source_joint->axis.x, source_joint->axis.y, source_joint->axis.z};
                const double axis_norm = joint.axis.stableNorm();
                if (!joint.axis.allFinite() || !std::isfinite(axis_norm) || axis_norm <= 1e-12)
                {
                    throw std::runtime_error("Invalid axis for joint: " + joint.name);
                }
                joint.axis /= axis_norm; // 只归一化，不改变所在坐标系。
            }

            /** 新增：把该关节明确声明的上下界保存到它自己的数据对象。 */
            const auto xml_joint = joint_xml_by_name.find(joint.name);
            if (xml_joint == joint_xml_by_name.end())
            {
                throw std::runtime_error("Joint missing from XML index: " + joint.name);
            }
            joint.position_bounds = readDeclaredPositionBounds(*source_joint, *xml_joint->second);

            result.joints.push_back(std::move(joint));
            current_link = robot->getLink(source_joint->parent_link_name);
            if (!current_link)
            {
                throw std::runtime_error("Parent link does not exist: " + source_joint->parent_link_name);
            }
        }

        /** 从 tip -> base 反转为 base -> tip，然后仅给活动关节分配 q 下标。 */
        std::reverse(result.joints.begin(), result.joints.end());
        for (auto &joint : result.joints)
        {
            if (joint.type == UrdfJointType::Fixed)
            {
                continue;
            }
            joint.q_index = result.joint_names.size();
            result.joint_names.push_back(joint.name);
        }
        if (result.joint_names.size() != expected_joint_count)
        {
            throw std::runtime_error(
                "Active joint count mismatch: expected " + std::to_string(expected_joint_count) +
                ", got " + std::to_string(result.joint_names.size()));
        }
        return result;
    }

} // namespace robot::motion