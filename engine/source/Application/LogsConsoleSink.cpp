/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsConsoleSink.h"

namespace {
    struct StreamMapping {
        const char* str;
        std::ostream* stream;
    };

    const StreamMapping streamMappings[] = {
        {"Output", &std::cout},
        {"Error", &std::cerr},
        {"Log", &std::clog}
    };

    std::ostream* Stream_fromString(const char* str) {
        const StreamMapping* iter = std::find_if(boost::begin(streamMappings), boost::end(streamMappings),
            boost::bind(strcmp, boost::bind(&StreamMapping::str, _1), str) == 0);
        if (iter != boost::end(streamMappings)) {
            return iter->stream;
        }
        return ROS_NULL;
    }
}

ros::LogsConsoleSink::LogsConsoleSink()
    : stream(ROS_NULL) {
}

bool ros::LogsConsoleSink::init(const PropertyTree& config) {
    if (!LogsSink::init(config)) {
        return false;
    }

    std::string streamStr = config.get<std::string>("Stream", "Error");
    stream = Stream_fromString(streamStr.c_str());
    if (!stream) {
        std::cerr << "Failed to initialize console sink: Unknown stream " << streamStr << std::endl;
        uninit();
        return false;
    }

    return true;
}

void ros::LogsConsoleSink::uninit() {
    stream = ROS_NULL;
    LogsSink::uninit();
}

bool ros::LogsConsoleSink::sendMessage(const LogMessage& message) {
    if (!stream) {
        return false;
    }
    (*stream) << message;
    return stream->good();
}

void ros::LogsConsoleSink::flushMessages() {
    if (stream) {
        stream->flush();
    }
}

