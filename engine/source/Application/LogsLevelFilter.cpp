/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include "LogsLevelFilter.h"

bool ros::LogsLevelFilter::Init(const PropertyTree& config) {
    StringOpt thresholdLevelConfig = config.get_optional<std::string>("ThresholdLevel");
    if (!thresholdLevelConfig) {
        std::cerr << "Missing threshold level in level filter config" << std::endl;
        return false;
    }

    LogLevelOpt thresholdLevel = LogLevel_FromString(thresholdLevelConfig->c_str());
    if (!thresholdLevel) {
        std::cerr << "Unknown threshold level " << *thresholdLevelConfig << " found in level filter config" << std::endl;
        return false;

    }

    this->thresholdLevel = thresholdLevel;
    return true;
}

bool ros::LogsLevelFilter::IsMessageAccepted(const LogMessage& message) const {
    if (!thresholdLevel) {
        return true;
    }
    return message.GetLevel() >= *thresholdLevel;
}
