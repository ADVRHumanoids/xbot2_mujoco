#include "../src/loading_utils.h"
#include <iostream>

#include "./config.h"
#define FDIR "@FILES_DIR@"

int main() {

    std::string outfname="centauro_xmj_test";
    LoadingUtils loader(outfname);
    std::string files_dir(FILES_DIR);

    fprintf(stderr, "[test_loading_utils]: loading test files from base dir at %s \n", files_dir.c_str());

    loader.set_urdf_path(files_dir + "/centauro.urdf");
    loader.set_simopt_path(files_dir + "/sim_opt.xml");
    loader.set_world_path(files_dir + "/world.xml");
    loader.set_sites_path(files_dir + "/sites.xml");
    
    loader.generate();
    
    std::string finalMuJoCoXML = loader.get_mj_xml();
    std::cout << finalMuJoCoXML << std::endl;

    return 0;
}