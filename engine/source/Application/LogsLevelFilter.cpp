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
#include "LogsLevelFilter.h"

bool ros::LogsLevelFilter::Init(const PropertyTree& config) {
    StringOpt thresholdLevelConfig = config.get_optional<String>("ThresholdLevel");
    if (!thresholdLevelConfig) {
        std::cerr << "Failed to initialize level filter: Missing threshold level" << std::endl;
        return false;
    }
    LogLevelOpt thresholdLevelMapping = LogLevel_FromString(thresholdLevelConfig->c_str());
    if (!thresholdLevelMapping) {
        std::cerr << "Failed to initialize level filter: Unknown threshold level" << *thresholdLevelConfig << std::endl;
        return false;
    }
    thresholdLevel = thresholdLevelMapping;

    return true;
}

void ros::LogsLevelFilter::Uninit() {
    thresholdLevel.reset();
}

bool ros::LogsLevelFilter::IsMessageAccepted(const LogMessage& message) const {
    if (!thresholdLevel) {
        return true;
    }
    return message.GetLevel() >= *thresholdLevel;
}
