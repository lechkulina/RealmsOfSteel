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
        LogLevel level;
        const char* str;
    } logLevelMappings[] = {
        {LogLevel_Trace, "Trace"},
        {LogLevel_Debug, "Debug"},
        {LogLevel_Warning, "Warning"},
        {LogLevel_Error, "Error"},
        {LogLevel_Critical, "Critical"}
    };
}

const char* ros::LogLevel_ToString(LogLevel level) {
    const LogLevelMapping* iter = std::find_if(boost::begin(logLevelMappings), boost::end(logLevelMappings),
                                               boost::bind(&LogLevelMapping::level, _1) == level);
    if (iter != boost::end(logLevelMappings)) {
        return iter->str;
    }
    return ROS_NULL;
}

ros::LogLevelOpt ros::LogLevel_FromString(const char* str) {
    const LogLevelMapping* iter = std::find_if(boost::begin(logLevelMappings), boost::end(logLevelMappings),
                                               boost::bind(strcmp, boost::bind(&LogLevelMapping::str, _1), str) == 0);
    if (iter != boost::end(logLevelMappings)) {
        return iter->level;
    }
    return LogLevelOpt();
}

std::ostream& ros::operator<<(std::ostream& stream, LogLevel level) {
    return stream << LogLevel_ToString(level);
}

std::ostream& ros::operator<<(std::ostream& stream, const LogMessage& message) {
    return stream << "[" << message.GetLevel() << "] " << message.str() << std::endl;
}
