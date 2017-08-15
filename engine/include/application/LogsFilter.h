/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_FILTER_H
#define ROS_LOGS_FILTER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <application/LogEntry.h>

namespace ros {
    class LogsFilter;
    typedef boost::shared_ptr<LogsFilter> LogsFilterPtr;
    typedef std::list<LogsFilterPtr> LogsFiltersList;

    class ROS_API LogsFilter : public boost::noncopyable {
        public:
            static LogsFilterPtr create(const std::string& classId);

            virtual ~LogsFilter() {}

            virtual bool init(const pt::ptree& config) =0;
            virtual bool isEntryAccepted(const LogEntry& entry) const =0;

        private:
            static Factory<LogsFilter> factory;
    };
}

#endif // ROS_LOGS_FILTER_H

