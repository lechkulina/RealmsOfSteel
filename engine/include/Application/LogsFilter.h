/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_FILTER_H
#define ROS_LOGS_FILTER_H

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <Core/PropertyTree.h>
#include <Core/Environment.h>
#include <Core/Factory.h>
#include <Application/LogMessage.h>

namespace ros {

    class LogsFilter;
    typedef boost::shared_ptr<LogsFilter> LogsFilterPtr;
    typedef Factory<String, LogsFilter> LogsFilterFactory;

    class ROS_API LogsFilter : public boost::noncopyable {
        public:
            static LogsFilterPtr Create(const PropertyTree& config);

            virtual ~LogsFilter() {}

            virtual bool Init(const PropertyTree& config) =0;
            virtual bool IsMessageAccepted(const LogMessage& message) const =0;

        private:
            static LogsFilterFactory factory;
    };

}

#endif // ROS_LOGS_FILTER_H

