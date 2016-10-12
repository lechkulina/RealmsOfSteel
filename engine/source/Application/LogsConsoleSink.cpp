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
#include "LogsConsoleSink.h"

namespace ros {

    static const struct StreamMapping {
       const char* str;
       std::ostream* stream;
    } streamMappings[] = {
        {"Output", &std::cout},
        {"Error", &std::cerr},
        {"Log", &std::clog}
    };

    static std::ostream* Stream_FromString(const char* str) {
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

bool ros::LogsConsoleSink::Init(const PropertyTree& config) {
    if (!LogsSink::Init(config)) {
        return false;
    }

    String streamConfig = config.get<String>("Stream", "Error");
    std::ostream* streamMapped = Stream_FromString(streamConfig.c_str());
    if (!streamMapped) {
        std::cerr << "Failed to initialize console sink: Unknown stream " << streamConfig << std::endl;
        return false;
    }
    stream = streamMapped;

    return true;
}

bool ros::LogsConsoleSink::SendMessage(const LogMessage& message) {
    if (!stream) {
        return false;
    }
    (*stream) << message;
    return stream->good();
}

void ros::LogsConsoleSink::FlushMessages() {
    if (stream) {
        stream->flush();
    }
}

