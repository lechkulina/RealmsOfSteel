/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include "LogsConsoleSink.h"

const std::string ros::LogsConsoleSink::DEFAULT_STREAM("output");

namespace {
    const struct StreamMapping {
        const char* str;
        std::ostream* stream;
    } streamMappings[] = {
        {"output", &std::cout},
        {"error", &std::cerr},
        {"log", &std::clog}
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

void ros::LogsConsoleSink::setStream(std::ostream* stream) {
    this->stream = stream;
}

bool ros::LogsConsoleSink::init(const pt::ptree& config) {
    if (!LogsSink::init(config)) {
        return false;
    }

    std::string streamStr = config.get("stream", DEFAULT_STREAM);
    std::ostream* stream = Stream_fromString(streamStr.c_str());
    if (!stream) {
        std::cerr << "Failed to initialize console sink: Unknown stream " << streamStr << std::endl;
        return false;
    }

    setStream(stream);
    return true;
}

bool ros::LogsConsoleSink::sendEntry(const LogEntry& entry) {
    if (!stream) {
        return false;
    }
    (*stream) << entry;
    return stream->good();
}

void ros::LogsConsoleSink::flushEntries() {
    if (stream) {
        stream->flush();
    }
}

