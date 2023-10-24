/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt
 All rights reserved.

 */

/*
 This Documentaion
 */

#include "resources.h"
#include <fstream>

std::string readFileContents(const char* filename) {
    std::ifstream ifs(filename);
    std::string result;
    ifs >> result;
    return result;
}
