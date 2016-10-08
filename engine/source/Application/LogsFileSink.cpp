/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsFileSink.h"

namespace ros {
    static const struct OpenModeMapping {
        const char* str;
        std::ios_base::openmode openMode;
    } openModesMapping[] = {
        {"Append", std::ios_base::out|std::ios_base::app},
        {"Truncate", std::ios_base::out|std::ios_base::trunc}
    };
}

bool ros::LogsFileSink::Init(const PropertyTree& config) {
    if (!LogsSink::Init(config)) {
        return false;
    }

    StringOpt filePathConfig = config.get_optional<std::string>("FilePath");
    if (!filePathConfig) {
        std::cerr << "Missing file path in file sink config" << std::endl;
        return false;
    }

    std::ios_base::openmode openMode = std::ios_base::out|std::ios_base::app;
    StringOpt openModeConfig = config.get_optional<std::string>("OpenMode");
    if (openModeConfig) {
        const OpenModeMapping* iter = std::find_if(boost::begin(openModesMapping), boost::end(openModesMapping),
                                                   boost::bind(&OpenModeMapping::str, _1) == *openModeConfig);
        if (iter != boost::end(openModesMapping)) {
            openMode = iter->openMode;
        } else {
            std::cerr << "Unknown open mode " << *openModeConfig << " found in file sink config" << std::endl;
        }
    }

    stream.open(filePathConfig->c_str(), openMode);
    if (!stream.good()) {
        std::cerr << " Failed to open file " << *filePathConfig << " for writing" << std::endl;
        return false;
    }

    return true;
}

bool ros::LogsFileSink::SendMessage(const LogMessage& message) {
    stream << message << std::endl;
    return stream.good();
}

void ros::LogsFileSink::FlushMessages() {
    stream.flush();
}


