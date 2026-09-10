#include "motion/kinematics/urdf_chain.hpp"

#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
using robot::motion::loadUrdfChain;
using robot::motion::UrdfJointType;

void expect(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error(message);
}

class TemporaryUrdf
{
public:
    explicit TemporaryUrdf(const std::string &xml)
    {
        char pattern[] = "/tmp/zk_urdf_chain_XXXXXX";
        const int fd = mkstemp(pattern);
        if (fd < 0) throw std::runtime_error("mkstemp failed");
        close(fd);
        path_ = pattern;
        std::ofstream file(path_, std::ios::binary);
        file << xml;
        file.close();
        if (!file)
        {
            unlink(path_.c_str());
            throw std::runtime_error("writing fixture failed");
        }
    }
    ~TemporaryUrdf() { unlink(path_.c_str()); }
    TemporaryUrdf(const TemporaryUrdf &) = delete;
    TemporaryUrdf &operator=(const TemporaryUrdf &) = delete;
    const std::string &path() const { return path_; }
private:
    std::string path_;
};

std::string replaceOnce(std::string text, const std::string &from, const std::string &to)
{
    const auto pos = text.find(from);
    if (pos == std::string::npos) throw std::runtime_error("bad test replacement: " + from);
    text.replace(pos, from.size(), to);
    return text;
}

int checks = 0;
void rejects(const std::string &xml, const std::string &message,
             const std::string &base = "base", const std::string &tip = "tool",
             std::size_t count = 2)
{
    TemporaryUrdf fixture(xml);
    try
    {
        (void)loadUrdfChain(fixture.path(), base, tip, count);
    }
    catch (const std::runtime_error &error)
    {
        expect(std::string(error.what()).find(message) != std::string::npos,
               std::string("wrong diagnostic: ") + error.what());
        ++checks;
        return;
    }
    throw std::runtime_error("invalid model was accepted: " + message);
}

const std::string kXml = R"(<robot name="example">
  <link name="base"/><link name="a"/><link name="b"/><link name="tool"/>
  <!-- Deliberately not in chain order. Geometry must come from joint/origin. -->
  <joint name="tool_mount" type="fixed">
    <parent link="b"/><child link="tool"/>
    <origin xyz="0 0 0.1" rpy="0 0 1.5707963267948966"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="a"/><child link="b"/>
    <axis xyz="0 2 0"/><origin xyz="0.2 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="a"/>
    <origin xyz="0 0 0.3"/>
    <limit lower="-2" upper="2" effort="10" velocity="1"/>
  </joint>
</robot>)";

void testSynthetic()
{
    TemporaryUrdf fixture(kXml);
    const auto chain = loadUrdfChain(fixture.path(), "base", "tool", 2);
    expect(chain.base_link == "base" && chain.tip_link == "tool", "wrong endpoints");
    expect(chain.joints.size() == 3 && chain.joint_names.size() == 2, "fixed joint count");
    expect(chain.joint_names[0] == "shoulder" && chain.joint_names[1] == "slide", "q order");
    expect(chain.joints[0].q_index == 0 && chain.joints[1].q_index == 1 &&
           !chain.joints[2].q_index, "q indices");
    expect(chain.joints[0].axis.isApprox(Eigen::Vector3d::UnitX()), "default axis");
    expect(chain.joints[1].axis.isApprox(Eigen::Vector3d::UnitY()), "axis normalization");
    expect(chain.joints[2].origin.position.isApprox(Eigen::Vector3d(0, 0, 0.1)), "TCP offset lost");
    expect((chain.joints[2].origin.orientation * Eigen::Vector3d::UnitX()).isApprox(
               Eigen::Vector3d::UnitY()), "RPY rotation");
    expect(chain.joints[0].origin.orientation.isApprox(Eigen::Quaterniond::Identity()), "default RPY");
    const auto subchain = loadUrdfChain(fixture.path(), "a", "tool", 1);
    expect(subchain.joint_names[0] == "slide" && subchain.joints.size() == 2, "subchain");
    ++checks;

    // Continuous support, missing origin defaults, negative axes, and full RPY composition.
    auto variants = replaceOnce(kXml, "type=\"revolute\"", "type=\"continuous\"");
    variants = replaceOnce(variants, "<origin xyz=\"0 0 0.3\"/>", "");
    variants = replaceOnce(variants, "0 2 0", "0 -2 0");
    variants = replaceOnce(variants, "0 0 1.5707963267948966", "0.3 -0.2 0.5");
    TemporaryUrdf variant_fixture(variants);
    const auto variant = loadUrdfChain(variant_fixture.path(), "base", "tool", 2);
    expect(variant.joints[0].type == UrdfJointType::Continuous, "continuous support");
    expect(variant.joints[0].origin.position.isZero(), "default position");
    expect(variant.joints[1].axis.isApprox(-Eigen::Vector3d::UnitY()), "axis sign");
    const double r = .3, p = -.2, y = .5;
    Eigen::Matrix3d expected_rotation;
    expected_rotation << std::cos(y)*std::cos(p),
        std::cos(y)*std::sin(p)*std::sin(r)-std::sin(y)*std::cos(r),
        std::cos(y)*std::sin(p)*std::cos(r)+std::sin(y)*std::sin(r),
        std::sin(y)*std::cos(p),
        std::sin(y)*std::sin(p)*std::sin(r)+std::cos(y)*std::cos(r),
        std::sin(y)*std::sin(p)*std::cos(r)-std::cos(y)*std::sin(r),
        -std::sin(p), std::cos(p)*std::sin(r), std::cos(p)*std::cos(r);
    expect(variant.joints[2].origin.orientation.toRotationMatrix().isApprox(expected_rotation),
           "nonzero RPY composition order");
    ++checks;

    rejects(kXml, "count mismatch", "base", "tool", 7);
    rejects(kXml, "does not exist", "missing");
    rejects(kXml, "does not exist", "base", "missing");
    rejects(kXml, "ancestor", "tool", "base");
    rejects(kXml, "distinct", "base", "base");
    rejects(kXml, "positive", "base", "tool", 0);
    rejects("<robot", "invalid XML");
    rejects("", "empty file");
    rejects("<world/>", "root element");
    rejects(replaceOnce(kXml, "0 2 0", "0 0 0"), "axis is zero");
    rejects(replaceOnce(kXml, "0 2 0", "0 nan 0"), "finite numbers");
    rejects(replaceOnce(kXml, "0 2 0", "0 inf 0"), "finite numbers");
    rejects(replaceOnce(kXml, "0 2 0", "0 2 0 1"), "finite numbers");
    rejects(replaceOnce(kXml, "0 2 0", "0 2"), "finite numbers");
    rejects(replaceOnce(kXml, "0 2 0", ""), "empty attribute");
    rejects(replaceOnce(kXml, "0 0 0.3", "0 0 inf"), "finite numbers");
    rejects(replaceOnce(kXml, "0 0 1.5707963267948966", "0 0 ${yaw}"), "finite numbers");
    rejects(replaceOnce(kXml, "type=\"revolute\"", "type=\"planar\""), "unsupported joint type");
    rejects(replaceOnce(kXml, "type=\"revolute\"", "type=\"floating\""), "unsupported joint type");
    rejects(replaceOnce(kXml, "type=\"revolute\"", "type=\"unknown\""), "unsupported joint type");
    rejects(replaceOnce(kXml, "<axis xyz=\"0 2 0\"/>", "<mimic joint=\"shoulder\"/>"), "mimic");
    rejects(replaceOnce(kXml, "<link name=\"a\"/>", "<link name=\"a\"/><link name=\"a\"/>"), "duplicate link");
    rejects(replaceOnce(kXml, "name=\"slide\"", "name=\"shoulder\""), "duplicate joint");
    rejects(replaceOnce(kXml, "<parent link=\"a\"/>", "<parent link=\"missing\"/>"), "unknown link");
    rejects(replaceOnce(kXml, "<child link=\"b\"/>", "<child link=\"a\"/>"), "self-loop");
    rejects(replaceOnce(kXml, "<child link=\"tool\"/>", "<child link=\"a\"/>"), "multiple parents");
    rejects(replaceOnce(kXml, "<parent link=\"base\"/>", "<parent link=\"tool\"/>"), "cyclic or disconnected");
    rejects(replaceOnce(kXml, "</robot>", "<link name=\"loose\"/></robot>"), "exactly one root");
    rejects(replaceOnce(kXml, "<origin xyz=\"0 0 0.3\"/>", "<origin/><origin/>"), "duplicate element");
    rejects(replaceOnce(kXml, "<parent link=\"base\"/>", ""), "missing element");
    rejects("<!DOCTYPE robot [<!ENTITY hidden 'secret'>]>" + kXml, "DTD");
    rejects(replaceOnce(kXml, "<origin xyz=\"0 0 0.3\"/>",
                        "<origin xmlns=\"urn:bad\" xyz=\"0 0 99\"/>"), "namespaced");
    rejects(replaceOnce(kXml, "<robot name=\"example\">", "<robot name=\"example\" xmlns=\"urn:bad\">"), "root element");
    rejects(replaceOnce(kXml, "</robot>", "<xacro:property name=\"length\" value=\"1\"/></robot>"), "xacro");
    rejects(std::string(8 * 1024 * 1024 + 1, ' '), "exceeds 8 MiB");

    // Rejecting unsupported chain geometry must not reject an unrelated branch.
    const auto branch_xml = replaceOnce(kXml, "</robot>", R"(
      <link name="finger"/>
      <joint name="finger_joint" type="revolute">
        <parent link="base"/><child link="finger"/><mimic joint="shoulder"/>
      </joint></robot>)");
    TemporaryUrdf branch_fixture(branch_xml);
    expect(loadUrdfChain(branch_fixture.path(), "base", "tool", 2).joint_names.size() == 2,
           "unselected branch changed the arm chain");
    ++checks;

    const auto missing_path = fixture.path() + ".does_not_exist";
    try
    {
        (void)loadUrdfChain(missing_path, "base", "tool", 2);
        throw std::logic_error("missing file accepted");
    }
    catch (const std::runtime_error &error)
    {
        expect(std::string(error.what()).find("cannot open") != std::string::npos, "missing file diagnostic");
        ++checks;
    }
}

void testT170c(const std::string &path)
{
    for (const std::string side : {"L", "R"})
    {
        const auto chain = loadUrdfChain(path, "WAIST_Y_S", side + "_WRIST_R_S", 7);
        const std::vector<std::string> suffixes{
            "SHOULDER_P", "SHOULDER_R", "SHOULDER_Y", "ELBOW_Y", "WRIST_P", "WRIST_Y", "WRIST_R"};
        expect(chain.joints.size() == 7, "unexpected T170C chain segments");
        for (std::size_t i = 0; i < 7; ++i)
            expect(chain.joint_names[i] == side + "_" + suffixes[i] && chain.joints[i].q_index == i,
                   "T170C q order mismatch");
        ++checks;
    }
}
} // namespace

int main(int argc, char **argv)
{
    try
    {
        testSynthetic();
        if (argc > 1) testT170c(argv[1]);
        std::cout << "URDF chain tests passed: " << checks << " cases; T170C file "
                  << (argc > 1 ? "checked" : "not provided") << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "URDF chain test failed: " << error.what() << '\n';
        return 1;
    }
}
