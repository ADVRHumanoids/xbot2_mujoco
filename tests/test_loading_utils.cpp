#include "../src/loading_utils.h"
#include <gtest/gtest.h>

#include "./config.h"
#define FDIR "@FILES_DIR@"

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

    std::string outfname="parsing_test2";
    LoadingUtils loader(outfname);
    std::string files_dir(FILES_DIR);

    fprintf(stderr, "[ParsingTest][GenerateURDF]: loading test files from base dir at %s \n", files_dir.c_str());
    fprintf(stderr, "[ParsingTest][GenerateURDF]: mesh root dir %s \n", mesh_root_directory.c_str());

    loader.set_mesh_rootdir(mesh_root_directory);
    loader.set_mesh_rootdir_subdirs(subdirs);
    loader.set_urdf_path(files_dir + "/centauro.urdf");
    loader.set_simopt_path(files_dir + "/sim_opt.xml");
    loader.set_world_path(files_dir + "/world.xml");
    loader.set_sites_path(files_dir + "/sites.xml");
    
    loader.generate();
    
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