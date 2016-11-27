/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsLevelFilter.h"

bool ros::LogsLevelFilter::init(const PropertyTree& config) {
    StringOpt thresholdLevelStr = config.get_optional<std::string>("ThresholdLevel");
    if (!thresholdLevelStr) {
        std::cerr << "Failed to initialize level filter: Missing threshold level" << std::endl;
        return false;
    }
    thresholdLevel = LogLevel_fromString(thresholdLevelStr->c_str());
    if (!thresholdLevel) {
        std::cerr << "Failed to initialize level filter: Unknown threshold level" << *thresholdLevelStr << std::endl;
        uninit();
        return false;
    }
    return true;
}

void ros::LogsLevelFilter::uninit() {
    thresholdLevel.reset();
}

bool ros::LogsLevelFilter::isMessageAccepted(const LogMessage& message) const {
    if (!thresholdLevel) {
        return true;
    }
    return message.getLevel() >= *thresholdLevel;
}
