/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_SINK_H
#define ROS_LOGS_SINK_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <application/LogMessage.h>
#include <application/LogsFilter.h>

namespace ros {
    class LogsSink;
    typedef boost::shared_ptr<LogsSink> LogsSinkPtr;
    typedef Factory<LogsSink> LogsSinkFactory;

    class ROS_API LogsSink : public LogsFilter {
        public:
            static LogsSinkPtr create(const PropertyTree& config);

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual bool isMessageAccepted(const LogMessage& message) const;

            virtual bool sendMessage(const LogMessage& message) =0;
            virtual void flushMessages() =0;

        private:
            typedef std::list<LogsFilterPtr> LogsFilterList;

            static LogsSinkFactory factory;
            LogsFilterList filters;
    };
}

#endif // ROS_LOGS_SINK_H

