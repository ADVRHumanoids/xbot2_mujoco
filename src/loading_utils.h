#ifndef LOADING_UTILS_H
#define LOADING_UTILS_H

#include <string>
#include <pugixml.hpp>

class LoadingUtils {
public:
    LoadingUtils(const std::string& name);

    void setURDFPath(const std::string& urdfPath);
    void setURDFCommand(const std::string& urdfCommand);
    void setSimOptPath(const std::string& simoptPath);
    void setWorldPath(const std::string& worldPath);
    void setCtrlCfgPath(const std::string& ctrlcfgPath);
    void setSitesPath(const std::string& sitesPath);

    std::string get_mj_xml(); // Public method to get the final MuJoCo XML
    void generate();
    std::string xml_path();

private:

    bool generated = false;
    std::string name_;
    std::string urdfPath_;
    std::string urdfCommand_;
    std::string simoptPath_;
    std::string worldPath_;
    std::string ctrlcfgPath_;
    std::string sitesPath_;

    std::string mjXmlDir_;
    std::string mjUrdfPath_;
    std::string mjXmlPath_;
    std::string mjXmlPathOrig_;

    std::string removeComments(const std::string& xml);
    std::string addMeshSimLinkBugFix(const std::string& urdf);
    void processURDF();
    void compileMuJoCoXML();
    void mergeXMLTrees(pugi::xml_node& parent, pugi::xml_node child);
    void mergeXML();
    void addSites();
};

#endif // LOADING_UTILS_H