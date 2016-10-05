/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include "LogsLevelFilter.h"

bool ros::LogsLevelFilter::Init(const PropertyTree& config) {
    StringOpt thresholdConfig = config.get_optional<std::string>("Threshold");
    if (thresholdConfig) {
        LogLevelOpt threshold = LogLevel_FromString(thresholdConfig->c_str());
        if (threshold) {
            this->threshold = threshold;
        } else {
            std::cerr << "Unknown threshold " << *thresholdConfig << " found in standard output sink config" << std::endl;
        }
    }
    return true;
}

bool ros::LogsLevelFilter::IsMessageAccepted(const LogMessage& message) const {
    if (!threshold) {
        return true;
    }
    return message.GetLevel() >= *threshold;
}
