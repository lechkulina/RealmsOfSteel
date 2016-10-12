/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <Application/LogMessage.h>

namespace ros {

    static const struct LogLevelMapping {
        const char* str;
        LogLevel level;
    } logLevelMappings[] = {
        {"Trace", LogLevel_Trace},
        {"Debug", LogLevel_Debug},
        {"Warning", LogLevel_Warning},
        {"Error", LogLevel_Error},
        {"Critical", LogLevel_Critical}
    };

}

ros::LogLevelOpt ros::LogLevel_FromString(const char* str) {
    const LogLevelMapping* iter = std::find_if(boost::begin(logLevelMappings), boost::end(logLevelMappings),
                                                 boost::bind(strcmp, boost::bind(&LogLevelMapping::str, _1), str) == 0);
    if (iter != boost::end(logLevelMappings)) {
        return iter->level;
    }
    return LogLevelOpt();
}


const char* ros::LogLevel_ToString(LogLevel level) {
    const LogLevelMapping* iter = std::find_if(boost::begin(logLevelMappings), boost::end(logLevelMappings),
                                                 boost::bind(&LogLevelMapping::level, _1) == level);
    if (iter != boost::end(logLevelMappings)) {
        return iter->str;
    }
    return ROS_NULL;
}

std::ostream& ros::operator<<(std::ostream& stream, LogLevel level) {
    return stream << LogLevel_ToString(level);
}

std::ostream& ros::operator<<(std::ostream& stream, const LogMessage& message) {
    return stream << "[" << message.GetLevel() << "] " << message.GetMessage() << std::endl;
}

ros::LogMessage::LogMessage(LogLevel level, const String& message)
    : level(level)
    , message(message)
    , timePoint(Clock::now()) {
}
