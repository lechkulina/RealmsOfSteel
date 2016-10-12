/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsFileSink.h"

namespace ros {

    typedef std::ios_base::openmode OpenMode;
    typedef boost::optional<OpenMode> OpenModeOpt;

    static const struct OpenModeMapping {
        const char* str;
        OpenMode openMode;
    } openModeMappings[] = {
        {"Append", std::ios_base::out|std::ios_base::app},
        {"Truncate", std::ios_base::out|std::ios_base::trunc}
    };

    static OpenModeOpt OpenMode_FromString(const char* str) {
        const OpenModeMapping* iter = std::find_if(boost::begin(openModeMappings), boost::end(openModeMappings),
                                                     boost::bind(strcmp, boost::bind(&OpenModeMapping::str, _1), str) == 0);
        if (iter != boost::end(openModeMappings)) {
            return iter->openMode;
        }
        return OpenModeOpt();
    }

}

bool ros::LogsFileSink::Init(const PropertyTree& config) {
    if (!LogsSink::Init(config)) {
        return false;
    }

    StringOpt filePathConfig = config.get_optional<String>("FilePath");
    if (!filePathConfig) {
        std::cerr << "Failed to initialize console sink: Missing file path" << std::endl;
        return false;
    }

    String openModeConfig = config.get<String>("OpenMode", "Truncate");
    OpenModeOpt openModeMapped = OpenMode_FromString(openModeConfig.c_str());
    if (!openModeMapped) {
        std::cerr << "Failed to initialize console sink: Unknown open mode " << openModeConfig << std::endl;
        return false;
    }

    stream.open(filePathConfig->c_str(), *openModeMapped);
    if (!stream.good()) {
        std::cerr << "Failed to initialize console sink: Unable to open file " << *filePathConfig << std::endl;
        return false;
    }

    return true;
}

bool ros::LogsFileSink::SendMessage(const LogMessage& message) {
    if (!stream.is_open()) {
        return false;
    }
    stream << message;
    return stream.good();
}

void ros::LogsFileSink::FlushMessages() {
    if (stream.is_open()) {
        stream.flush();
    }
}


