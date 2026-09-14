#include "motion/kinematics/urdf_chain.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
    // 上游：启动程序时，从命令行提供 URDF 文件路径。
    if (argc != 2)
    {
        std::cerr
            << "Usage: motion_urdf_chain_test <robot.urdf>\n";
        return 2;
    }

    try
    {
        const std::string urdf_path = argv[1];

        // 1. 调用我们自己的加载函数，提取 T170C 左臂。
        const auto chain = robot::motion::loadUrdfChain(
            urdf_path,
            "WAIST_Y_S",
            "L_WRIST_R_S",
            7);

        // 2. 根据这份具体 URDF，预期得到以下活动关节顺序。
        const std::vector<std::string> expected_joint_names{
            "L_SHOULDER_P",
            "L_SHOULDER_R",
            "L_SHOULDER_Y",
            "L_ELBOW_Y",
            "L_WRIST_P",
            "L_WRIST_Y",
            "L_WRIST_R"};

        if (chain.joint_names != expected_joint_names)
        {
            throw std::runtime_error(
                "Unexpected active joint names or order");
        }

        // 3. 打印运动链概况。
        std::cout
            << "Base link: " << chain.base_link << '\n'
            << "Tip link: " << chain.tip_link << '\n'
            << "Total joints: " << chain.joints.size() << '\n'
            << "Active joints: " << chain.joint_names.size() << '\n';

        // 以下浮点数按小数点后六位显示。
        std::cout << std::fixed << std::setprecision(6);

        // 4. 按 base -> tip 顺序，打印每个关节的结构数据。
        for (const auto &joint : chain.joints)
        {
            const auto &position = joint.origin.position;
            const auto &orientation = joint.origin.orientation;

            std::cout
                << "\nJoint: " << joint.name << '\n'
                << "  Parent: " << joint.parent_link << '\n'
                << "  Child: " << joint.child_link << '\n';

            std::cout
                << "  Origin xyz [m]: "
                << position.x() << ' '
                << position.y() << ' '
                << position.z() << '\n';

            // 明确打印顺序是 w、x、y、z。
            std::cout
                << "  Origin quaternion [w x y z]: "
                << orientation.w() << ' '
                << orientation.x() << ' '
                << orientation.y() << ' '
                << orientation.z() << '\n';

            if (joint.q_index.has_value())
            {
                std::cout
                    << "  Input: q[" << *joint.q_index << "]\n"
                    << "  Axis in joint frame: "
                    << joint.axis.x() << ' '
                    << joint.axis.y() << ' '
                    << joint.axis.z() << '\n';
            }
            else
            {
                std::cout
                    << "  Fixed joint: no q input\n";
            }
        }

        std::cout << "\nURDF chain loading and order check passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "URDF chain test failed: "
            << error.what() << '\n';

        return 1;
    }
}