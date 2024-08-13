#include "../src/loading_utils.h"
#include <iostream>

#include "./config.h"
#define FDIR "@FILES_DIR@"

int main() {

    std::string outfname="centauro_xmj_test";
    LoadingUtils loader(outfname);
    std::string filesDir(FILES_DIR);

    fprintf(stderr, "[test_loading_utils]: loading test files from base dir at %s \n", filesDir.c_str());

    loader.setURDFPath(filesDir + "/centauro.urdf");
    loader.setSimOptPath(filesDir + "/sim_opt.xml");
    loader.setWorldPath(filesDir + "/world.xml");
    loader.setSitesPath(filesDir + "/sites.xml");

    std::string finalMuJoCoXML = loader.get_mj_xml();
    std::cout << finalMuJoCoXML << std::endl;

    return 0;
}