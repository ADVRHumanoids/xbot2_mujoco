#include "loading_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <filesystem>
#include <pugixml.hpp>

LoadingUtils::LoadingUtils(const std::string& name)
    : name_(name) {
    mjXmlDir_ = "/tmp/" + name + "_mujoco";
    mjUrdfPath_ = mjXmlDir_ + "/" + name + ".urdf";
    mjXmlPath_ = mjXmlDir_ + "/" + name + ".xml";
    mjXmlPathOrig_ = mjXmlDir_ + "/" + name + ".orig.xml";

    // Create directory
    std::filesystem::remove_all(mjXmlDir_);
    std::filesystem::create_directories(mjXmlDir_);
}

void LoadingUtils::setURDFPath(const std::string& urdfPath) {
    urdfPath_ = urdfPath;
}

void LoadingUtils::setURDFCommand(const std::string& urdfCommand) {
    urdfCommand_ = urdfCommand;
}

void LoadingUtils::setSimOptPath(const std::string& simoptPath) {
    simoptPath_ = simoptPath;
}

void LoadingUtils::setWorldPath(const std::string& worldPath) {
    worldPath_ = worldPath;
}

void LoadingUtils::setCtrlCfgPath(const std::string& ctrlcfgPath) {
    ctrlcfgPath_ = ctrlcfgPath;
}

void LoadingUtils::setSitesPath(const std::string& sitesPath) {
    sitesPath_ = sitesPath;
}

std::string LoadingUtils::removeComments(const std::string& xml) {
    pugi::xml_document doc;
    doc.load_string(xml.c_str());
    for (pugi::xml_node node = doc.first_child(); node; node = node.next_sibling()) {
        for (pugi::xml_node child = node.first_child(); child; child = child.next_sibling()) {
            if (child.type() == pugi::node_comment) {
                node.remove_child(child);
            }
        }
    }

    pugi::xml_node robot = doc.child("robot");
    pugi::xml_node mujoco = robot.append_child("mujoco");
    pugi::xml_node compiler = mujoco.append_child("compiler");
    compiler.append_attribute("fusestatic").set_value("false");

    std::ostringstream oss;
    doc.save(oss);
    return oss.str();
}

void LoadingUtils::createSymlink(const std::string& src, const std::string& dst) {
    std::filesystem::remove(dst);
    std::filesystem::create_symlink(src, dst);
}

void LoadingUtils::processURDF() {
    std::string urdf;
    if (!urdfPath_.empty()) {
        std::ifstream urdfFile(urdfPath_);
        urdf = std::string((std::istreambuf_iterator<char>(urdfFile)), std::istreambuf_iterator<char>());
    } else if (!urdfCommand_.empty()) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(urdfCommand_.c_str(), "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        urdf = result;
    } else {
        throw std::runtime_error("Either URDF path or URDF command must be specified");
    }

    urdf = removeComments(urdf);

    std::ofstream outFile(mjUrdfPath_);
    outFile << urdf;
    outFile.close();
}

void LoadingUtils::compileMuJoCoXML() {
    std::string cmd = "mujoco_compile " + mjUrdfPath_ + " " + mjXmlPathOrig_;
    std::system(cmd.c_str());
}

void LoadingUtils::mergeXMLTrees(pugi::xml_node& parent, pugi::xml_node& child) {
    for (pugi::xml_node& childNode : child.children()) {
        pugi::xml_node existingNode = parent.child(childNode.name());
        if (existingNode && childNode.first_child()) {
            mergeXMLTrees(existingNode, childNode);
        } else {
            parent.append_copy(childNode);
        }
    }
}

void LoadingUtils::mergeXML() {
    pugi::xml_document mjXmlDoc;
    pugi::xml_document simOptDoc;
    pugi::xml_document worldDoc;

    mjXmlDoc.load_file(mjXmlPathOrig_.c_str());
    simOptDoc.load_file(simoptPath_.c_str());
    worldDoc.load_file(worldPath_.c_str());

    mjXmlDoc.child("mujoco").remove_child("compiler");
    mjXmlDoc.child("mujoco").remove_child("size");

    mergeXMLTrees(mjXmlDoc.child("mujoco"), simOptDoc.child("mujoco"));
    mergeXMLTrees(mjXmlDoc.child("mujoco"), worldDoc.child("mujoco"));

    mjXmlDoc.save_file(mjXmlPath_.c_str());
}

void LoadingUtils::addSites() {
    pugi::xml_document mjXmlDoc;
    pugi::xml_document sitesDoc;

    mjXmlDoc.load_file(mjXmlPath_.c_str());
    sitesDoc.load_file(sitesPath_.c_str());

    for (pugi::xml_node siteBody : sitesDoc.children("body")) {
        std::string siteName = siteBody.attribute("name").value();
        pugi::xml_node bodyNode = mjXmlDoc.child("mujoco").find_child_by_attribute("body", "name", siteName.c_str());
        if (bodyNode) {
            pugi::xml_node siteNode = siteBody.child
