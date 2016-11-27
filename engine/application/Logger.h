/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGGER_H
#define ROS_LOGGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <application/LogsSink.h>

namespace ros {
    class Logger;
    typedef boost::shared_ptr<Logger> LoggerPtr;

    class ROS_API Logger : public LogsSink {
        public:
            static LoggerPtr create(const PropertyTree& config);
            static LoggerPtr getInstance() { return logger; }

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual bool sendMessage(const LogMessage& message);
            virtual void flushMessages();

            static bool report(LogLevel level, const boost::format& format) {
                return getInstance()->sendMessage(LogMessage(level, format.str()));
            }

        protected:
            typedef std::list<LogsSinkPtr> LogsSinkList;
            LogsSinkList sinks;

        private:
            static LoggerPtr logger;
    };
}

#endif // ROS_LOGGER_H

