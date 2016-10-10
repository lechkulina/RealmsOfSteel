/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOG_MESSAGE_H
#define ROS_LOG_MESSAGE_H

#include <ostream>
#include <boost/optional.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>
#include <Core/Common.h>
#include <Core/Environment.h>
#include <Core/PropertyTree.h>

namespace ros {

    enum LogLevel {
        LogLevel_Trace,
        LogLevel_Debug,
        LogLevel_Warning,
        LogLevel_Error,
        LogLevel_Critical
    };

    typedef boost::optional<LogLevel> LogLevelOpt;

    const char* LogLevel_ToString(LogLevel level);
    LogLevelOpt LogLevel_FromString(const char* str);
    std::ostream& operator<<(std::ostream& stream, LogLevel level);

    typedef boost::format LogFormat;

    class ROS_API LogMessage {
            typedef boost::chrono::system_clock Clock;

        public:
            typedef Clock::time_point TimePoint;

            LogMessage(LogLevel level, const String& message);

            LogLevel GetLevel() const { return level; }
            const String& GetMessage() const { return message; }
            TimePoint GetTimePoint() const { return timePoint; }

        private:
            LogLevel level;
            String message;
            TimePoint timePoint;
    };

    std::ostream& operator<<(std::ostream& stream, const LogMessage& message);

}

#endif // ROS_LOG_MESSAGE_H

