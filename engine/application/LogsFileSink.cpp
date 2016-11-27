/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsFileSink.h"

namespace {
    typedef boost::optional<std::ios_base::openmode> OpenModeOpt;
    struct OpenModeMapping {
        const char* str;
        std::ios_base::openmode openMode;
    };

    const OpenModeMapping openModeMappings[] = {
        {"Append", std::ios_base::out|std::ios_base::app},
        {"Truncate", std::ios_base::out|std::ios_base::trunc}
    };

    OpenModeOpt OpenMode_fromString(const char* str) {
        const OpenModeMapping* iter = std::find_if(boost::begin(openModeMappings), boost::end(openModeMappings),
            boost::bind(strcmp, boost::bind(&OpenModeMapping::str, _1), str) == 0);
        if (iter != boost::end(openModeMappings)) {
            return iter->openMode;
        }
        return OpenModeOpt();
    }
}

ros::LogsFileSink::~LogsFileSink() {
    uninit();
}

bool ros::LogsFileSink::init(const PropertyTree& config) {
    if (!LogsSink::init(config)) {
        return false;
    }

    StringOpt filePath = config.get_optional<std::string>("FilePath");
    if (!filePath) {
        std::cerr << "Failed to initialize console sink: Missing file path" << std::endl;
        uninit();
        return false;
    }

    std::string openModeStr = config.get<std::string>("OpenMode", "Truncate");
    OpenModeOpt openMode = OpenMode_fromString(openModeStr.c_str());
    if (!openMode) {
        std::cerr << "Failed to initialize console sink: Unknown open mode " << openModeStr << std::endl;
        uninit();
        return false;
    }

    stream.open(filePath->c_str(), *openMode);
    if (!stream.good()) {
        std::cerr << "Failed to initialize console sink: Unable to open file " << *filePath << std::endl;
        uninit();
        return false;
    }

    return true;
}

void ros::LogsFileSink::uninit() {
    if (stream.is_open()) {
        stream.close();
    }
    LogsSink::uninit();
}

bool ros::LogsFileSink::sendMessage(const LogMessage& message) {
    if (!stream.is_open()) {
        return false;
    }
    stream << message;
    return stream.good();
}

void ros::LogsFileSink::flushMessages() {
    if (stream.is_open()) {
        stream.flush();
    }
}


