//
//  utils.hpp
//  superimpose-heatmap
//
//  Created by Supannee Tanathong on 22/01/2025.
//

#include <iostream>
#include <string>
#include <tuple>

std::tuple<std::string, std::string, std::string> ExtractFromFilePath(const std::string& filePath) {
    size_t idx1 = filePath.find_last_of('.');
    size_t idx2 = filePath.find_last_of('/');
    if (idx1 == std::string::npos || idx2 == std::string::npos) {
        return {"", "", ""};
    }
    std::string extension = filePath.substr(idx1+1); // exclude .
    std::string fileName = filePath.substr(idx2+1, idx1 - idx2 - 1);
    std::string path = filePath.substr(0, idx2); // exclude /
    return {path, fileName, extension};
}


