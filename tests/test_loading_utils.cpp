#include "../src/loading_utils.h"
#include <gtest/gtest.h>

#include "./config.h"
#define FDIR "@FILES_DIR@"

#include <vector>
std::string IIT_CENTAURO_ROS_PKG_ROOT="";

class ParsingTest : public ::testing::Test {
protected:

    ParsingTest()
    {
    
    }

    void SetUp() override {
        
    }

    void TearDown() override {


    }

};

TEST_F(ParsingTest, GenerateURDFWithRootDir) {

    std::string mesh_root_directory = IIT_CENTAURO_ROS_PKG_ROOT+"/"+ "centauro_urdf/meshes";
    std::vector<std::string> subdirs = {"v2", "realsense", "simple"}; // to preserver mesh subdir 
    // when adding root to URDF

    std::string outfname="GenerateURDFWithRootDir";
    LoadingUtils loader(outfname);
    std::string files_dir(FILES_DIR);

    fprintf(stdout, "[ParsingTest][GenerateURDF]: loading test files from base dir at %s \n", files_dir.c_str());
    fprintf(stdout, "[ParsingTest][GenerateURDF]: mesh root dir %s \n", mesh_root_directory.c_str());

    loader.set_mesh_rootdir(mesh_root_directory);
    loader.set_mesh_rootdir_subdirs(subdirs);
    loader.set_urdf_path(files_dir + "/centauro.urdf");
    loader.set_simopt_path(files_dir + "/sim_opt.xml");
    loader.set_world_path(files_dir + "/world.xml");
    loader.set_sites_path(files_dir + "/sites.xml");
    
    loader.set_xbot_config_path(files_dir+ "/xbot2_basic.yaml");

    std::string srdf_xbot_path = loader.get_srdf_path_fromxbotconfig();
    std::vector<std::string> other_jnt_list = {"knee_pitch_2", "ankle_pitch_3", "j_arm1_4",
        "other_joint"};
    
    std::map<std::string, double> retrieved_homing = loader.generate_homing_map(other_jnt_list);
    for (const auto& pair : retrieved_homing) {
        std::cout << "Joint: " << pair.first << ", Value: " << pair.second << std::endl;
    }

    std::vector<double> ordered_homing = loader.generate_homing_from_list(other_jnt_list);
    for (size_t i = 0; i < other_jnt_list.size(); ++i) {
        std::cout << "Joint: " << other_jnt_list[i] << ", Homing Value: " << ordered_homing[i] << std::endl;
    }

    loader.generate();
    
    std::string finalMuJoCoXML = loader.get_mj_xml();
    std::cout << finalMuJoCoXML << std::endl;
}

TEST_F(ParsingTest, TestHomingParsing) {

    std::string outfname="TestHomingParsing";
    LoadingUtils loader(outfname);
    std::string files_dir(FILES_DIR);

    fprintf(stdout, "[ParsingTest][GenerateURDF]: loading test files from base dir at %s \n", files_dir.c_str());
    
    loader.set_xbot_config_path(files_dir+ "/xbot2_basic.yaml");

    std::string srdf_xbot_path = loader.get_srdf_path_fromxbotconfig();
    std::vector<std::string> other_jnt_list = {"knee_pitch_2", "ankle_pitch_3", "j_arm1_4",
        "other_joint"};
    
    std::cout << "\nhoming map:" << std::endl;
    std::map<std::string, double> retrieved_homing = loader.generate_homing_map(other_jnt_list);
    for (const auto& pair : retrieved_homing) {
        std::cout << "Joint: " << pair.first << ", Value: " << pair.second << std::endl;
    }

    std::cout << "\nordered homing list:" << std::endl;
    std::vector<double> ordered_homing = loader.generate_homing_from_list(other_jnt_list);
    for (size_t i = 0; i < other_jnt_list.size(); ++i) {
        std::cout << "Joint: " << other_jnt_list[i] << ", Homing Value: " << ordered_homing[i] << std::endl;
    }
    
    std::string finalMuJoCoXML = loader.get_mj_xml();
    std::cout << finalMuJoCoXML << std::endl;
}

int main(int argc, char** argv) {

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--iit-centauro-ros-pkg-root" && i + 1 < argc) {
            IIT_CENTAURO_ROS_PKG_ROOT = argv[++i];
        }
    }

    // Ensure that root_dir is provided
    if (IIT_CENTAURO_ROS_PKG_ROOT.empty()) {
        std::cerr << "Error: --iit-centauro-ros-pkg-root argument is mandatory." << std::endl;
        return 1; // Exit with error code
    }

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}