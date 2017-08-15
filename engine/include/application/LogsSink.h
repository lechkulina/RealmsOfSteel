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
#include <application/LogEntry.h>
#include <application/LogsFilter.h>

namespace ros {
    class LogsSink;
    typedef boost::shared_ptr<LogsSink> LogsSinkPtr;
    typedef std::list<LogsSinkPtr> LogsSinksList;

    class ROS_API LogsSink : public LogsFilter {
        public:
            static LogsSinkPtr create(const std::string& classId);

            virtual bool init(const pt::ptree& config);
            virtual bool isEntryAccepted(const LogEntry& entry) const;

            void addFilter(LogsFilterPtr filter);

            virtual bool sendEntry(const LogEntry& entry) =0;
            virtual void flushEntries() =0;

        private:
            static Factory<LogsSink> factory;
            LogsFiltersList filters;
    };
}

#endif // ROS_LOGS_SINK_H

