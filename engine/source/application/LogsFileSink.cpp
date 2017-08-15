/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsFileSink.h"

const ros::fs::path ros::LogsFileSink::DEFAULT_PATH("logs.txt");
const std::string ros::LogsFileSink::DEFAULT_OPEN_MODE("truncate");

namespace {
    const struct OpenModeMapping {
        const char* str;
        std::ios_base::openmode openMode;
    } openModeMappings[] = {
        {"append", std::ios_base::out|std::ios_base::app},
        {"truncate", std::ios_base::out|std::ios_base::trunc}
    };

    ros::OpenModeOpt OpenMode_fromString(const char* str) {
        const OpenModeMapping* iter = std::find_if(boost::begin(openModeMappings), boost::end(openModeMappings),
            boost::bind(strcmp, boost::bind(&OpenModeMapping::str, _1), str) == 0);
        if (iter != boost::end(openModeMappings)) {
            return iter->openMode;
        }
        return ros::OpenModeOpt();
    }
}

ros::LogsFileSink::~LogsFileSink() {
    close();
}

bool ros::LogsFileSink::open(const fs::path& path, std::ios_base::openmode openMode) {
    close();
    stream.open(path.string().c_str(), openMode);
    if (!stream.good()) {
        std::cerr << "Failed to initialize file sink: Unable to open file " << path << std::endl;
        close();
        return false;
    }
    this->path = path;
    this->openMode = openMode;
    return true;
}

void ros::LogsFileSink::close() {
    stream.close();
    path.clear();
    openMode.reset();
}

bool ros::LogsFileSink::init(const pt::ptree& config) {
    if (!LogsSink::init(config)) {
        return false;
    }

    fs::path path = config.get("path", DEFAULT_PATH);
    if (path.empty()) {
        std::cerr << "Failed to initialize console sink: Missing file path" << std::endl;
        return false;
    }

    std::string openModeStr = config.get("open-mode", DEFAULT_OPEN_MODE);
    OpenModeOpt openMode = OpenMode_fromString(openModeStr.c_str());
    if (!openMode) {
        std::cerr << "Failed to initialize console sink: Unknown open mode " << openModeStr << std::endl;
        return false;
    }

    return open(path, *openMode);
}

bool ros::LogsFileSink::sendEntry(const LogEntry& entry) {
    if (!stream.is_open()) {
        return false;
    }
    stream << entry;
    return stream.good();
}

void ros::LogsFileSink::flushEntries() {
    if (stream.is_open()) {
        stream.flush();
    }
}


