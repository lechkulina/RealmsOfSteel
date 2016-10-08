/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <iostream>
#include "LogsConsoleSink.h"

ros::LogsConsoleSink::LogsConsoleSink()
    : stream(&std::cerr) {
}

bool ros::LogsConsoleSink::Init(const PropertyTree& config) {
    if (!LogsSink::Init(config)) {
        return false;
    }
    StringOpt streamConfig = config.get_optional<std::string>("Stream");
    if (streamConfig) {
        if (*streamConfig == "Output") {
            stream = &std::cout;
        } else if (*streamConfig == "Error") {
            stream = &std::cerr;
        } else {
            std::cerr << "Unknown stream " << *streamConfig << " found in standard output sink config" << std::endl;
        }
    }
    return true;
}

bool ros::LogsConsoleSink::SendMessage(const LogMessage& message) {
    (*stream) << message;
    return stream->good();
}

void ros::LogsConsoleSink::FlushMessages() {
    stream->flush();
}

