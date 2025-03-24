#include "loading_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <filesystem>
#include <unordered_map>

LoadingUtils::LoadingUtils(const std::string& name)
    : name(name) {
    mjxml_dir = "/tmp/" + name + "_mujoco";
    mjurdf_path = mjxml_dir + "/" + name + ".urdf";
    mjxml_path = mjxml_dir + "/" + name + ".mjcf";
    mjxml_path_orig = mjxml_dir + "/" + name + ".orig.xml";

    // Create directory
    std::filesystem::remove_all(mjxml_dir);
    std::filesystem::create_directories(mjxml_dir);
}

void LoadingUtils::set_mesh_rootdir(const std::string& mesh_root_dir) {
    mesh_rootdir=mesh_root_dir;
    if (mesh_rootdir=="None") {
        use_custom_mesh_rootdir=false;
    } else {
        use_custom_mesh_rootdir=true;
    }
}

void LoadingUtils::set_mesh_rootdir_subdirs(const std::vector<std::string>& mesh_rootsubdirs) {
    mesh_root_subdirs= mesh_rootsubdirs;
}

void LoadingUtils::set_urdf_path(const std::string& urdfpath) {
    urdf_path = urdfpath;
}

void LoadingUtils::set_srdf_path(const std::string& srdfpath) {
    srdf_path = srdfpath;
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

void LoadingUtils::set_sites_path(const std::string& sitespath) {
    sites_path = sitespath;
}

void LoadingUtils::set_xbot_config_path(const std::string& configpath) {
    xbot_config_path = configpath;
}

std::string LoadingUtils::get_xbot_config_path() {
    return xbot_config_path;
}

std::string LoadingUtils::get_srdf_path_fromxbotconfig(const std::string xbot_cf_path) {
    // Load the YAML file
    std::string srdf_path = "";
    if (!xbot_cf_path.empty()) {
        YAML::Node config = YAML::LoadFile(xbot_cf_path);
    
        // Extract the srdf_path from the YAML structure
        auto srdf_path_item = config["XBotInterface"]["srdf_path"];
        if (srdf_path_item) {
            srdf_path = srdf_path_item.as<std::string>();
            // Replace occurrences of $PWD with the directory of xbot_cf_path
            std::string xbot_dir = std::filesystem::path(xbot_cf_path).parent_path().string();
            std::string::size_type pos = 0;
            while ((pos = srdf_path.find("$PWD", pos)) != std::string::npos) {
                srdf_path.replace(pos, 4, xbot_dir);
                pos += xbot_dir.length();
            }
        }
        
    }

    return srdf_path;
}

std::string LoadingUtils::get_urdf_path_fromxbotconfig(const std::string xbot_cf_path) {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(xbot_cf_path);
    std::string urdf_path = "";
    auto urdf_item = config["XBotInterface"]["urdf_path"];
    if (urdf_item) {
        urdf_path = config["XBotInterface"]["urdf_path"].as<std::string>();
        // Replace occurrences of $PWD with the directory of xbot_cf_path
        std::string xbot_dir = std::filesystem::path(xbot_cf_path).parent_path().string();
        std::string::size_type pos = 0;
        while ((pos = urdf_path.find("$PWD", pos)) != std::string::npos) {
            urdf_path.replace(pos, 4, xbot_dir);
            pos += xbot_dir.length();
        }
    }

    return urdf_path;
}

std::map<std::string, double> LoadingUtils::get_homing_from_srdf(const std::string& srdf_path) {
    std::map<std::string, double> homing_map;

    // Load the SRDF file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(srdf_path.c_str());

    if (!result) {
        fprintf(stderr,"[LoadingUtils][get_homing_from_srdf]: failed to load SRDF file at \"%s\" with error \"%s\" \n", 
            srdf_path.c_str(), result.description());
        return homing_map;
    }

    // Find the <group_state> tag with group="chains" and name="home"
    pugi::xml_node robot = doc.child("robot");
    if (!robot) {
        std::cerr << "No <robot> element found in SRDF file: " << srdf_path << std::endl;
        return homing_map;
    }

    for (pugi::xml_node group_state : robot.children("group_state")) {
        std::string group_attr = group_state.attribute("group").as_string();
        std::string name_attr = group_state.attribute("name").as_string();

        if (group_attr == "chains" && name_attr == "home") {
            // Found the correct group_state, now extract joints
            for (pugi::xml_node joint : group_state.children("joint")) {
                std::string joint_name = joint.attribute("name").as_string();
                double joint_value = joint.attribute("value").as_double();
                homing_map[joint_name] = joint_value;
            }
            break;  // Exit the loop after finding the correct group_state
        }
    }

    return homing_map;
}

std::map<std::string, double> LoadingUtils::generate_homing_map(const std::vector<std::string>& jnt_name_list, 
        std::string srdfpath,
        const std::string xbot_cf_path,
        double fallback_val) {
    
    if (srdfpath.empty()) { // then we try to read it from xbot config file
        printf( "[LoadingUtils][generate_homing_map]: will try to retrieve homing configuration from xbot2 config file \"%s\" \n", xbot_cf_path.c_str());
        srdfpath = get_srdf_path_fromxbotconfig(xbot_cf_path);
    } else {
        printf( "[LoadingUtils][generate_homing_map]: will try to retrieve homing configuration SRDF file \"%s\" \n", srdfpath.c_str());
    }
    
    std::map<std::string, double> result;

    if (!srdfpath.empty()) {
        std::map<std::string, double> homing_map = get_homing_from_srdf(srdfpath);
        
        for (const std::string& jnt_name : jnt_name_list) {
            // Check if the joint name exists in the homing map
            if (homing_map.find(jnt_name) != homing_map.end()) {
                result[jnt_name] = homing_map[jnt_name];
            } else {
                // Default if the joint name is not found in the homing map
                result[jnt_name] = fallback_val;
            }
        }
    }

    return result;
}

std::map<std::string, double> LoadingUtils::generate_homing_map(std::string srdfpath,
    const std::string xbot_cf_path) {

    if (srdfpath.empty()) { // then we try to read it from xbot config file
        printf( "[LoadingUtils][generate_homing_map]: will try to retrieve homing configuration from xbot2 config file \"%s\" \n", xbot_cf_path.c_str());
        srdfpath = get_srdf_path_fromxbotconfig(xbot_cf_path);
    } else {
        printf( "[LoadingUtils][generate_homing_map]: will try to retrieve homing configuration SRDF file \"%s\" \n", srdfpath.c_str());
    }

    std::map<std::string, double> homing_map;

    if (!srdfpath.empty()) {
        homing_map=get_homing_from_srdf(srdfpath);
    }

    return homing_map;
}

std::vector<double> LoadingUtils::generate_homing_from_list(const std::vector<std::string>& jnt_name_list,
        std::string srdfpath,
        const std::string xbot_cf_path,
        double fallback_val) {

    if (srdfpath.empty()) { // then we try to read it from xbot config file
        printf( "[LoadingUtils][generate_homing_from_list]: will try to retrieve homing configuration from xbot2 config file \"%s\" \n", xbot_cf_path.c_str());
        srdfpath = get_srdf_path_fromxbotconfig(xbot_cf_path);
    } else {
        printf( "[LoadingUtils][generate_homing_from_list]: will try to retrieve homing configuration SRDF file \"%s\" \n", srdfpath.c_str());
    }

    std::vector<double> result;

    if (!srdfpath.empty()) {
        std::map<std::string, double> homing_map = get_homing_from_srdf(srdfpath);
        for (const std::string& jnt_name : jnt_name_list) {
            // Check if the joint name exists in the homing map
            if (homing_map.find(jnt_name) != homing_map.end()) {
                result.push_back(homing_map[jnt_name]);
            } else {
                // Default to 0 if the joint name is not found in the homing map
                result.push_back(0.0);
            }
        }
    }

    return result;
}

std::tuple<std::vector<std::string>, std::vector<double>> LoadingUtils::generate_ordered_homing(std::string srdfpath,
    const std::string xbot_cf_path) {

    if (srdfpath.empty()) { // then we try to read it from xbot config file
        printf( "[LoadingUtils][generate_ordered_homing]: will try to retrieve homing configuration from xbot2 config file \"%s\" \n", xbot_cf_path.c_str());
        srdfpath = get_srdf_path_fromxbotconfig(xbot_cf_path);
    } else {
        printf( "[LoadingUtils][generate_ordered_homing]: will try to retrieve homing configuration SRDF file \"%s\" \n", srdfpath.c_str());
    }

    std::map<std::string, double> homing_map = get_homing_from_srdf(srdfpath);

    std::vector<std::string> jnt_names;
    std::vector<double> homing_vals;

    // Iterate over the map and collect keys
    for (const auto& homing : homing_map) {
        jnt_names.push_back(homing.first);
        homing_vals.push_back(homing.second);
    }

    return std::make_tuple(jnt_names, homing_vals);
}

void LoadingUtils::print_homing(std::map<std::string, double> homing_map) {
    // Print joint names
    printf("[LoadingUtils][print_homing]: joint names\n");
    printf("[");
    
    // Iterate over the map to print the keys (joint names)
    std::size_t count = 0;
    for (const auto& pair : homing_map) {
        printf("%s", pair.first.c_str());  // Print joint name (key)
        if (count < homing_map.size() - 1) {
            printf(", ");  // Add a comma except for the last element
        }
        ++count;
    }
    printf("]:\n");

    // Print homing values
    printf("[LoadingUtils][print_homing]: homing values\n");
    printf("[");
    
    count = 0;
    for (const auto& pair : homing_map) {
        printf("%.2f", pair.second);  // Print homing value (value)
        if (count < homing_map.size() - 1) {
            printf(", ");  // Add a comma except for the last element
        }
        ++count;
    }
    printf("]\n");
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

    std::ostringstream oss;
    doc.save(oss);
    return oss.str();
}

std::string LoadingUtils::apply_mesh_root(const std::string& urdf, const std::string& mesh_rootdir, const std::vector<std::string>& subdir_names) {
    std::string processedUrdf;
    std::string::size_type lastPos = 0;
    std::string search_pattern = "<mesh filename=\"";
    int pattern_length = search_pattern.length();
    std::string::size_type pos = urdf.find(search_pattern, lastPos);

    while (pos != std::string::npos) {
        std::string::size_type uriStart = pos + pattern_length;
        std::string::size_type uriEnd = urdf.find("\"", uriStart);
        if (uriEnd == std::string::npos) {
            processedUrdf += urdf.substr(lastPos);
            break;
        }

        std::string originalPath = urdf.substr(uriStart, uriEnd - uriStart);

        // Extract the filename and the subdirectory part
        std::string filename;
        std::string subdirPart;
        std::string::size_type slashPos = originalPath.find_last_of('/');

        if (slashPos != std::string::npos) {
            filename = originalPath.substr(slashPos + 1);
            subdirPart = originalPath.substr(0, slashPos);
        } else {
            filename = originalPath;
            subdirPart = ""; // No subdir part
        }

        // Determine the new path
        std::string newPath = mesh_rootdir + "/" + filename;

        // Check if the original path contains any of the specified subdir_names
        for (const auto& subdir : subdir_names) {
            std::string::size_type subdirPos = subdirPart.find(subdir);
            if (subdirPos != std::string::npos) {
                // Calculate the portion of the path from the subdirectory to the filename
                std::string newSubdirPart = subdirPart.substr(subdirPos);
                newPath = mesh_rootdir +  "/" + newSubdirPart + "/" + filename;
                break;
            }
        }

        // Replace the filename in the URDF with the new path
        processedUrdf += urdf.substr(lastPos, uriStart - lastPos) + newPath + "\"";

        lastPos = uriEnd + 1;
        pos = urdf.find(search_pattern, lastPos);
    }

    // Add the remaining part of the URDF string
    if (lastPos < urdf.length()) {
        processedUrdf += urdf.substr(lastPos);
    }

    return processedUrdf;
}

std::string LoadingUtils::add_mesh_simlink_bfix(const std::string& urdf) {
    std::string processedUrdf;
    std::string::size_type lastPos = 0;
    std::string search_pattern="<mesh filename=\"";
    int pattern_length=16;
    std::string::size_type pos = urdf.find(search_pattern, lastPos);

    std::unordered_map<std::string, int> meshCounter; // For generating unique filenames
    std::string tempDir = mjxml_dir; // Using temporary directory for symbolic links

    // Ensure temporary directory exists
    std::filesystem::create_directories(tempDir);

    while (pos != std::string::npos) {
        std::string::size_type uriStart = pos + pattern_length; // Length of "filename=\""
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
        pos = urdf.find(search_pattern, lastPos);
    }

    // Add the remaining part of the URDF string
    if (lastPos < urdf.length()) {
        processedUrdf += urdf.substr(lastPos);
    }

    return processedUrdf;
}

void LoadingUtils::preprocess_urdf() {
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
    printf( "[LoadingUtils][mergeXML]: removed comments from URDF \n");

    if (use_custom_mesh_rootdir) {
        urdf = apply_mesh_root(urdf,mesh_rootdir,mesh_root_subdirs);
        printf( "[LoadingUtils][mergeXML]: applying new mesh root dir %s to URDF \n", mesh_rootdir.c_str());
    }

    urdf = add_mesh_simlink_bfix(urdf); // bug fix for mujoco not allowing identical mesh paths on multiple bodies
    printf( "[LoadingUtils][mergeXML]: created mesh simlink to circumvent mujoco_compile BUG.\n");

    std::ofstream outFile(mjurdf_path);
    outFile << urdf;
    printf( "[LoadingUtils][mergeXML]: dumped modified URDF at %s \n", mjurdf_path.c_str());
    outFile.close();
}

void LoadingUtils::compile_urdf_for_mj(std::string xml, std::string outpath) {
    printf( "[LoadingUtils][mergeXML]: compiling URDF using mujoco_compile...\n");
    std::string cmd = "mujoco_compile " + mjurdf_path + " " + mjxml_path_orig;
    auto ret = std::system(cmd.c_str());
    if (ret != 0) {
        // If the return value is non-zero, there was an error
        printf( "[LoadingUtils][compile_urdf_for_mj]: Error occurred during mujoco_compile. Return code: %d\n", ret);
    } else {
        printf( "[LoadingUtils][mergeXML]: done. Return code: %d\n", ret);
    }
}

void LoadingUtils::merg_xml_trees(pugi::xml_node& parent, pugi::xml_node child) {
    // Iterate over each child node in the 'child' XML document
    for (pugi::xml_node childNode : child.children()) {
        // Check if the parent has a corresponding child node with the same name
        pugi::xml_node found = parent.find_child_by_attribute(childNode.name(), "name", childNode.attribute("name").value());

        if (found) {
            // If a matching node is found, check if it's a complex node that needs merging
            if (childNode.first_child()) {
                // If the child node has children, recursively merge their contents
                merg_xml_trees(found, childNode);
            }
        } else {
            // If no matching node is found, append a copy of the child node as a sibling
            parent.append_copy(childNode);
        }
    }
}

void LoadingUtils::merge_xml(std::string to_be_added, std::string into, std::string out) {
    // Load the MuJoCo XML, simulator options XML, and world XML documents
    pugi::xml_document inTo;
    pugi::xml_document toBeAdded;

    inTo.load_file(into.c_str());
    toBeAdded.load_file(to_be_added.c_str());
    printf( "[LoadingUtils][mergeXML]: loaded xml at %s \n", to_be_added.c_str());

    // Remove specific nodes from the MuJoCo XML
    pugi::xml_node mujocoNode = inTo.child("mujoco");
    // mujocoNode.remove_child("compiler");
    // mujocoNode.remove_child("size");

    // Merge the simulator options and world XML into the main MuJoCo XML
    merg_xml_trees(mujocoNode, toBeAdded.child("mujoco"));

    // Save the merged XML back to a file
    inTo.save_file(out.c_str());
}

// Function to find a <body> node with a specific name attribute using XPath
pugi::xml_node find_body_by_name_with_xpath(const pugi::xml_node& parent, const std::string& body_name) {
    // Construct XPath query to find <body> nodes with the specific name attribute value
    std::string xpath_query = "//body[@name='" + body_name + "']";
    // Perform the XPath query on the parent node
    pugi::xml_node result = parent.select_node(xpath_query.c_str()).node();
    return result;
}

void LoadingUtils::add_sites() {
    pugi::xml_document mjXmlDoc;
    pugi::xml_document sitesDoc;

    std::cout << "Loading sites and MuJoCo XML..." << std::endl;
    // Load the MuJoCo XML and sites XML documents
    if (!mjXmlDoc.load_file(mjxml_path.c_str())) {
        std::cerr << "[LoadingUtils][add_sites]: Error loading MuJoCo XML file at " << mjxml_path << std::endl;
        return;
    }
    if (!sitesDoc.load_file(sites_path.c_str())) {
        std::cerr << "[LoadingUtils][add_sites]: Error loading sites XML file at " << sites_path << std::endl;
        return;
    }
    printf( "[LoadingUtils][add_sites]: Loaded sites from %s\n", sites_path.c_str());

    // Find the root node of the sites XML document
    pugi::xml_node sitesRoot = sitesDoc.child("sites");
    if (!sitesRoot) {
        std::cerr << "[LoadingUtils][add_sites]: No <sites> root element found in " << sites_path << std::endl;
        return;
    }

    // Iterate over all <body> elements in the sites XML
    for (pugi::xml_node siteBody : sitesRoot.children("body")) {
        std::string siteName = siteBody.attribute("name").value();
        // Find the corresponding <body> element in the MuJoCo XML using XPath
        pugi::xml_node bodyNode = find_body_by_name_with_xpath(mjXmlDoc.child("mujoco"), siteName);
        
        if (bodyNode) {
            // Iterate over all <site> elements under the <body> in the sites XML
            for (pugi::xml_node siteNode : siteBody.children("site")) {
                // Add each <site> to the corresponding <body> node in the MuJoCo XML
                bodyNode.append_copy(siteNode);
            }
        } else {
            std::cerr << "[LoadingUtils][add_sites]: Body with name " << siteName << " not found in MuJoCo XML" << std::endl;
        }
    }

    // Save the modified MuJoCo XML document
    if (!mjXmlDoc.save_file(mjxml_path.c_str())) {
        std::cerr << "[LoadingUtils][add_sites]: Error saving modified MuJoCo XML file to " << mjxml_path << std::endl;
    } else {
        std::cout << "Successfully added sites and saved MuJoCo XML." << std::endl;
    }
}

void LoadingUtils::add_compiler_opts() {
    printf( "[LoadingUtils][mergeXML]: Adding compiler opts to URDF's xml...\n");
    pugi::xml_document preprocessed_urdf;
    preprocessed_urdf.load_file(mjurdf_path.c_str());
    pugi::xml_node root = preprocessed_urdf.document_element();
    pugi::xml_node robot = preprocessed_urdf.child("robot");
    pugi::xml_node mujoco = robot.append_child("mujoco");
    pugi::xml_node compiler = mujoco.append_child("compiler");
    compiler.append_attribute("discardvisual").set_value("false");
    compiler.append_attribute("strippath").set_value("false");
    compiler.append_attribute("angle").set_value("radian");
    compiler.append_attribute("boundmass").set_value(0.001);
    compiler.append_attribute("boundinertia").set_value(0.0001);
    compiler.append_attribute("balanceinertia").set_value(true);
    compiler.append_attribute("fusestatic").set_value("false");
    // Save the modified XML back to the file
    if (!preprocessed_urdf.save_file(mjurdf_path.c_str())) {
        printf("[Error] Failed to save modified URDF file: %s\n", mjurdf_path.c_str());
    } else {
        printf("[LoadingUtils][mergeXML]: Compiler opts added successfully.\n");
    }
}

void LoadingUtils::generate() {
    preprocess_urdf(); // removed comments + some bfixes + adds compiler opts
    add_compiler_opts(); // load modified URDF and add compiler options, then dump
    compile_urdf_for_mj(mjurdf_path, mjxml_path_orig); // compile urdf
    merge_xml(simopt_path, mjxml_path_orig, 
        mjxml_path); // merge sim opts 
    merge_xml(world_path, mjxml_path, 
        mjxml_path); // merge world
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
