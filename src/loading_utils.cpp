#include "loading_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <filesystem>
#include <unordered_map>

LoadingUtils::LoadingUtils(const std::string& name)
    : name_(name) {
    mjxml_dir = "/tmp/" + name + "_mujoco";
    mjurdf_path = mjxml_dir + "/" + name + ".urdf";
    mjxml_path = mjxml_dir + "/" + name + ".xml";
    mjxml_path_orig = mjxml_dir + "/" + name + ".orig.xml";

    // Create directory
    std::filesystem::remove_all(mjxml_dir);
    std::filesystem::create_directories(mjxml_dir);
}

void LoadingUtils::set_urdf_path(const std::string& urdfpath) {
    urdf_path = urdfpath;
}

void LoadingUtils::set_urdf_cmd(const std::string& urdfcommand) {
    urdf_command = urdfcommand;
}

void LoadingUtils::set_simopt_path(const std::string& simoptpath) {
    simopt_path = simoptpath;
}

void LoadingUtils::set_world_path(const std::string& worldpath) {
    world_path = worldpath;
}

void LoadingUtils::set_ctrlcfg_path(const std::string& ctrlcfgpath) {
    ctrlcfg_path = ctrlcfgpath;
}

void LoadingUtils::set_sites_path(const std::string& sitespath) {
    sites_path = sitespath;
}

std::string LoadingUtils::remove_comments(const std::string& xml) {
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

std::string LoadingUtils::add_mesh_simlink_bfix(const std::string& urdf) {
    std::string processedUrdf;
    std::string::size_type lastPos = 0;
    std::string::size_type pos = urdf.find("filename=\"", lastPos);

    std::unordered_map<std::string, int> meshCounter; // For generating unique filenames
    std::string tempDir = mjxml_dir; // Using temporary directory for symbolic links

    // Ensure temporary directory exists
    std::filesystem::create_directories(tempDir);

    while (pos != std::string::npos) {
        std::string::size_type uriStart = pos + 10; // Length of "filename=\""
        std::string::size_type uriEnd = urdf.find("\"", uriStart);
        if (uriEnd == std::string::npos) {
            // If there's no closing quote, append the rest of the string and exit
            processedUrdf += urdf.substr(lastPos);
            break;
        }
        std::string uri = urdf.substr(uriStart, uriEnd - uriStart);

        // Generate a unique filename for each mesh file
        std::string filename = std::filesystem::path(uri).filename().string();
        std::string uniqueFileName = std::to_string(meshCounter[filename]++) + "_" + filename;
        std::string dstFile = tempDir + "/" + uniqueFileName;

        // Create symbolic link if not exists
        if (!std::filesystem::exists(dstFile)) {
            std::filesystem::remove(dstFile); // Remove existing symlink if any
            std::filesystem::create_symlink(uri, dstFile);
        }

        // Replace the filename in the URDF with the unique filename
        processedUrdf += urdf.substr(lastPos, uriStart - lastPos) + dstFile + "\"";

        lastPos = uriEnd + 1; // Move past the closing quote
        pos = urdf.find("filename=\"", lastPos);
    }

    // Add the remaining part of the URDF string
    if (lastPos < urdf.length()) {
        processedUrdf += urdf.substr(lastPos);
    }

    return processedUrdf;
}

void LoadingUtils::process_urdf() {
    std::string urdf;
    if (!urdf_path.empty()) {
        std::ifstream urdfFile(urdf_path);
        urdf = std::string((std::istreambuf_iterator<char>(urdfFile)), std::istreambuf_iterator<char>());
    } else if (!urdf_command.empty()) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(urdf_command.c_str(), "r"), pclose);
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

    urdf = remove_comments(urdf);
    fprintf(stderr, "[LoadingUtils][mergeXML]: removed comments from URDF \n");
    urdf = add_mesh_simlink_bfix(urdf); // bug fix for mujoco not allowing identical mesh paths on multiple bodies
    fprintf(stderr, "[LoadingUtils][mergeXML]: created mesh simlink to circumvent mujoco_compile BUG.\n");

    std::ofstream outFile(mjurdf_path);
    outFile << urdf;
    fprintf(stderr, "[LoadingUtils][mergeXML]: dumped modified URDF at %s \n", mjurdf_path.c_str());
    outFile.close();
}

void LoadingUtils::compile_mujoco_xml() {
    fprintf(stderr, "[LoadingUtils][mergeXML]: compiling URDF using mujoco_compile...\n");
    std::string cmd = "mujoco_compile " + mjurdf_path + " " + mjxml_path_orig;
    auto ret = std::system(cmd.c_str());
    if (ret != 0) {
        // If the return value is non-zero, there was an error
        fprintf(stderr, "[LoadingUtils][compile_mujoco_xml]: Error occurred during mujoco_compile. Return code: %d\n", ret);
    } else {
        fprintf(stderr, "[LoadingUtils][mergeXML]: done. Return code: %d\n", ret);
    }

}

void LoadingUtils::merg_xml_trees(pugi::xml_node& parent, pugi::xml_node child) {
    // Iterate over each child node in the 'child' XML document
    for (pugi::xml_node childNode : child.children()) {
        // Check if the parent has a corresponding child node
        pugi::xml_node found = parent.child(childNode.name());
        if (found) {
            // If a matching node is found, recursively merge their children
            merg_xml_trees(found, childNode);
        } else {
            // If no matching node is found, append a copy of the child node
            parent.append_copy(childNode);
        }
    }
}

void LoadingUtils::merge_xml() {
    // Load the MuJoCo XML, simulator options XML, and world XML documents
    pugi::xml_document mjXmlDoc;
    pugi::xml_document simOptDoc;
    pugi::xml_document worldDoc;

    mjXmlDoc.load_file(mjxml_path_orig.c_str());
    simOptDoc.load_file(simopt_path.c_str());
    fprintf(stderr, "[LoadingUtils][mergeXML]: loaded xml at %s \n", simopt_path.c_str());
    worldDoc.load_file(world_path.c_str());
    fprintf(stderr, "[LoadingUtils][mergeXML]: loaded xml at %s \n", world_path.c_str());

    // Remove specific nodes from the MuJoCo XML
    pugi::xml_node mujocoNode = mjXmlDoc.child("mujoco");
    mujocoNode.remove_child("compiler");
    mujocoNode.remove_child("size");

    // Merge the simulator options and world XML into the main MuJoCo XML
    merg_xml_trees(mujocoNode, simOptDoc.child("mujoco"));
    merg_xml_trees(mujocoNode, worldDoc.child("mujoco"));

    // Save the merged XML back to a file
    mjXmlDoc.save_file(mjxml_path.c_str());
}

void LoadingUtils::add_sites() {
    pugi::xml_document mjXmlDoc;
    pugi::xml_document sitesDoc;

    mjXmlDoc.load_file(mjxml_path.c_str());
    sitesDoc.load_file(sites_path.c_str());
    fprintf(stderr, "[LoadingUtils][mergeXML]: loaded sites at %s \n", sites_path.c_str());

    for (pugi::xml_node siteBody : sitesDoc.children("body")) {
        std::string siteName = siteBody.attribute("name").value();
        pugi::xml_node bodyNode = mjXmlDoc.child("mujoco").find_child_by_attribute("body", "name", siteName.c_str());
        if (bodyNode) {
            pugi::xml_node siteNode = siteBody.child("site");
            bodyNode.append_copy(siteNode);
        }
    }

    mjXmlDoc.save_file(mjxml_path.c_str());
}

void LoadingUtils::generate() {
    process_urdf();
    compile_mujoco_xml();
    merge_xml();
    add_sites();
    generated=true;
}

std::string LoadingUtils::xml_path() {
    return mjxml_path;
}

std::string LoadingUtils::get_mj_xml() {
    if (generated) {
        std::ifstream finalXmlFile(mjxml_path);
        std::stringstream finalXmlBuffer;
        finalXmlBuffer << finalXmlFile.rdbuf();
        return finalXmlBuffer.str();
    } else {
        return "";
    }
}
