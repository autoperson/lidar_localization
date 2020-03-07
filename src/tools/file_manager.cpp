/*
 * @Description: creae file or directory, read and write file
 * @Author: Ren Qian
 * @Date: 2020-02-24 20:09:32
 */
#include "lidar_localization/tools/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace lidar_localization {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if (!ofs) {
        LOG(WARNING) << "create file failed: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }

    return true;
}

bool FileManager::InitDirectory(std::string directory_path, std::string use_for) {
    if (boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::remove_all(directory_path);
    }

    return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "create directory failed: " << std::endl << directory_path << std::endl << std::endl;
        return false;
    }

    std::cout << use_for << " save path is ：" << std::endl << directory_path << std::endl << std::endl;
    return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "create directory failed: " << std::endl << directory_path << std::endl << std::endl;
        return false;
    }

    return true;
}
}