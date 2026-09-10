#include "motion/kinematics/urdf_chain.hpp"

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace robot::motion
{
namespace
{
[[noreturn]] void fail(const std::string &message)
{
    throw std::runtime_error("URDF chain: " + message);
}

bool named(const xmlNode *node, const char *name)
{
    return node && node->type == XML_ELEMENT_NODE && !node->ns &&
           xmlStrEqual(node->name, BAD_CAST name);
}

std::string attribute(xmlNode *node, const char *name, bool required = false)
{
    xmlChar *raw = xmlGetProp(node, BAD_CAST name);
    if (!raw)
    {
        if (required) fail(std::string("missing attribute '") + name + "'");
        return {};
    }
    const std::unique_ptr<xmlChar, decltype(xmlFree)> value(raw, xmlFree);
    const std::string result(reinterpret_cast<const char *>(value.get()));
    if (required && result.empty()) fail(std::string("empty attribute '") + name + "'");
    return result;
}

xmlNode *child(xmlNode *node, const char *name, bool required = false)
{
    xmlNode *result = nullptr;
    for (auto *entry = node->children; entry; entry = entry->next)
    {
        if (!named(entry, name)) continue;
        if (result) fail(std::string("duplicate element '") + name + "'");
        result = entry;
    }
    if (required && !result) fail(std::string("missing element '") + name + "'");
    return result;
}

Eigen::Vector3d vectorAttribute(xmlNode *node, const char *name,
                                const Eigen::Vector3d &fallback)
{
    if (!node || !xmlHasProp(node, BAD_CAST name)) return fallback;
    std::istringstream input(attribute(node, name, true));
    input.imbue(std::locale::classic());
    Eigen::Vector3d result;
    std::string extra;
    if (!(input >> result.x() >> result.y() >> result.z()) ||
        (input >> extra) || !result.allFinite())
        fail(std::string("expected three finite numbers in '") + name + "'");
    return result;
}

std::string readLocalFile(const std::string &path)
{
    constexpr std::streamoff kMaxBytes = 8 * 1024 * 1024;
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) fail("cannot open local file: " + path);
    const auto size = input.tellg();
    if (size <= 0 || size > kMaxBytes) fail("empty file or file exceeds 8 MiB: " + path);
    std::string text(static_cast<std::size_t>(size), '\0');
    input.seekg(0);
    if (!input.read(text.data(), static_cast<std::streamsize>(text.size())))
        fail("cannot read local file: " + path);
    // UTF-8 / ASCII URDF only. Reject DTD before XML parsing; never expand custom
    // entities or perform network access. Also reject unexpanded xacro input.
    if (text.find('\0') != std::string::npos ||
        text.find("<!DOCTYPE") != std::string::npos ||
        text.find("<!ENTITY") != std::string::npos ||
        text.find("xacro:") != std::string::npos)
        fail("NUL, DTD, entities or unexpanded xacro are not supported");
    return text;
}

struct Tree
{
    std::unordered_set<std::string> links;
    std::unordered_map<std::string, xmlNode *> parent_joint;
};

Tree indexTree(xmlNode *robot)
{
    Tree tree;
    std::unordered_set<std::string> joint_names;
    std::unordered_map<std::string, std::vector<std::string>> children;
    for (auto *node = robot->children; node; node = node->next)
    {
        if (node->type == XML_ELEMENT_NODE && node->ns)
            fail("namespaced robot elements are not supported");
        if (named(node, "link") && !tree.links.insert(attribute(node, "name", true)).second)
            fail("duplicate link name");
    }
    if (tree.links.empty()) fail("no links found");
    for (auto *node = robot->children; node; node = node->next)
    {
        if (!named(node, "joint")) continue;
        const auto name = attribute(node, "name", true);
        if (!joint_names.insert(name).second) fail("duplicate joint: " + name);
        (void)attribute(node, "type", true);
        const auto parent = attribute(child(node, "parent", true), "link", true);
        const auto next = attribute(child(node, "child", true), "link", true);
        if (!tree.links.count(parent) || !tree.links.count(next))
            fail("joint refers to an unknown link: " + name);
        if (parent == next || !tree.parent_joint.emplace(next, node).second)
            fail("self-loop or multiple parents at joint: " + name);
        children[parent].push_back(next);
    }
    std::vector<std::string> roots;
    for (const auto &link : tree.links)
        if (!tree.parent_joint.count(link)) roots.push_back(link);
    if (roots.size() != 1) fail("model must have exactly one root link");
    std::unordered_set<std::string> visited;
    while (!roots.empty())
    {
        const auto link = roots.back();
        roots.pop_back();
        if (!visited.insert(link).second) fail("cyclic model");
        for (const auto &next : children[link]) roots.push_back(next);
    }
    if (visited.size() != tree.links.size()) fail("cyclic or disconnected model");
    return tree;
}

UrdfChainJoint readJoint(xmlNode *node)
{
    UrdfChainJoint joint;
    joint.name = attribute(node, "name", true);
    joint.parent_link = attribute(child(node, "parent", true), "link", true);
    joint.child_link = attribute(child(node, "child", true), "link", true);
    if (child(node, "mimic")) fail("mimic joint is not supported: " + joint.name);
    const auto type = attribute(node, "type", true);
    if (type == "fixed") joint.type = UrdfJointType::Fixed;
    else if (type == "revolute") joint.type = UrdfJointType::Revolute;
    else if (type == "continuous") joint.type = UrdfJointType::Continuous;
    else if (type == "prismatic") joint.type = UrdfJointType::Prismatic;
    else fail("unsupported joint type '" + type + "': " + joint.name);

    auto *origin = child(node, "origin");
    joint.origin.position = vectorAttribute(origin, "xyz", Eigen::Vector3d::Zero());
    const auto rpy = vectorAttribute(origin, "rpy", Eigen::Vector3d::Zero());
    // URDF fixed-axis roll/pitch/yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    joint.origin.orientation = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
    joint.origin = normalizedPose(joint.origin);
    if (joint.type != UrdfJointType::Fixed)
    {
        joint.axis = vectorAttribute(child(node, "axis"), "xyz", Eigen::Vector3d::UnitX());
        const double length = joint.axis.stableNorm();
        if (!std::isfinite(length) || length <= 1e-12)
            fail("joint axis is zero or invalid: " + joint.name);
        joint.axis /= length;
    }
    return joint;
}
} // namespace

UrdfChain loadUrdfChain(const std::string &urdf_path,
                       const std::string &base_link,
                       const std::string &tip_link,
                       const std::size_t expected_joint_count)
{
    if (base_link.empty() || tip_link.empty() || base_link == tip_link || expected_joint_count == 0)
        fail("provide distinct base/tip links and a positive joint count");

    // 1. 初始化时读取 XML，生命周期限于这次加载，返回值不引用 XML 内存。
    const auto text = readLocalFile(urdf_path);
    using Document = std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)>;
    const Document document(xmlReadMemory(text.data(), static_cast<int>(text.size()),
                                         nullptr, "UTF-8", XML_PARSE_NONET |
                                         XML_PARSE_NOERROR | XML_PARSE_NOWARNING), &xmlFreeDoc);
    if (!document) fail("invalid XML: " + urdf_path);
    auto *robot = xmlDocGetRootElement(document.get());
    if (!named(robot, "robot")) fail("root element must be <robot>");
    (void)attribute(robot, "name", true);
    std::vector<xmlNode *> pending{robot};
    while (!pending.empty())
    {
        auto *node = pending.back();
        pending.pop_back();
        if (node->type == XML_ELEMENT_NODE && node->ns)
            fail("namespaced XML elements are not supported");
        for (auto *entry = node->children; entry; entry = entry->next)
            pending.push_back(entry);
    }

    // 2. 检查树结构，并查明每个连杆的父关节。
    const auto tree = indexTree(robot);
    if (!tree.links.count(base_link) || !tree.links.count(tip_link))
        fail("base or tip link does not exist");

    // 3. 从 tip 沿父关节向回找 base，再反转为真正的 base -> tip 顺序。
    UrdfChain result{base_link, tip_link, {}, {}};
    auto current = tip_link;
    while (current != base_link)
    {
        const auto parent = tree.parent_joint.find(current);
        if (parent == tree.parent_joint.end()) fail("base is not an ancestor of tip");
        auto joint = readJoint(parent->second);
        current = joint.parent_link;
        result.joints.push_back(std::move(joint));
    }
    std::reverse(result.joints.begin(), result.joints.end());

    // 4. fixed 参与坐标变换但不占 q；其他关节按链顺序分配 q_index。
    for (auto &joint : result.joints)
    {
        if (joint.type == UrdfJointType::Fixed) continue;
        joint.q_index = result.joint_names.size();
        result.joint_names.push_back(joint.name);
    }
    if (result.joint_names.size() != expected_joint_count)
        fail("active joint count mismatch: expected " + std::to_string(expected_joint_count) +
             ", got " + std::to_string(result.joint_names.size()));
    return result;
}
} // namespace robot::motion
