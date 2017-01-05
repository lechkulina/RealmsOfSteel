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
#include <application/LogMessage.h>

namespace ros {
    class LogsFilter;
    typedef boost::shared_ptr<LogsFilter> LogsFilterPtr;
    typedef Factory<LogsFilter> LogsFilterFactory;

    class ROS_API LogsFilter : public boost::noncopyable {
        public:
            static LogsFilterPtr create(const PropertyTree& config);

            virtual ~LogsFilter() {}

            virtual bool init(const PropertyTree& config) =0;
            virtual void uninit() =0;
            virtual bool isMessageAccepted(const LogMessage& message) const =0;

        private:
            static LogsFilterFactory factory;
    };
}

#endif // ROS_LOGS_FILTER_H

