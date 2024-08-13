#ifndef LOADING_UTILS_H
#define LOADING_UTILS_H

#include <string>

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

private:
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
    void createSymlink(const std::string& src, const std::string& dst);
    void processURDF();
    void compileMuJoCoXML();
    void mergeXML();
    void addSites();
};

#endif // LOADING_UTILS_H