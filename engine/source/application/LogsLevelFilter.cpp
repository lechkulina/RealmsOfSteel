/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsLevelFilter.h"

void ros::LogsLevelFilter::setThreshold(LogLevelOpt threshold) {
    this->threshold = threshold;
}

bool ros::LogsLevelFilter::init(const pt::ptree& config) {
    StringOpt thresholdStr = config.get_optional<std::string>("threshold");
    if (!thresholdStr) {
        std::cerr << "Failed to initialize level filter: Missing threshold level" << std::endl;
        return false;
    }
    LogLevelOpt threshold = LogLevel_fromString(thresholdStr->c_str());
    if (!threshold) {
        std::cerr << "Failed to initialize level filter: Unknown threshold level" << *thresholdStr << std::endl;
        return false;
    }
    setThreshold(threshold);
    return true;
}

bool ros::LogsLevelFilter::isEntryAccepted(const LogEntry& entry) const {
    if (!threshold) {
        return true;
    }
    return entry.getLevel() >= *threshold;
}
