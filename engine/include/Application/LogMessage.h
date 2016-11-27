/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOG_MESSAGE_H
#define ROS_LOG_MESSAGE_H

#include <boost/format.hpp>
#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum LogLevel {
        LogLevel_Trace,
        LogLevel_Debug,
        LogLevel_Warning,
        LogLevel_Error,
        LogLevel_Critical
    };

    typedef boost::optional<LogLevel> LogLevelOpt;

    LogLevelOpt LogLevel_fromString(const char* str);
    const char* LogLevel_toString(LogLevel level);
    std::ostream& operator<<(std::ostream& stream, LogLevel level);

    class ROS_API LogMessage {
        public:
            LogMessage(LogLevel level, const std::string& message);

            LogLevel getLevel() const { return level; }
            const std::string& getMessage() const { return message; }
            SystemClock::time_point getTimePoint() const { return timePoint; }

        private:
            LogLevel level;
            std::string message;
            SystemClock::time_point timePoint;
    };

    std::ostream& operator<<(std::ostream& stream, const LogMessage& message);
}

#endif // ROS_LOG_MESSAGE_H

