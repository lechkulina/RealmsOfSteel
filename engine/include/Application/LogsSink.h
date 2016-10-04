/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_SINK_H
#define ROS_LOGS_SINK_H

#include <string>
#include <list>
#include <Core/Environment.h>
#include <Core/Factory.h>
#include <Application/LogMessage.h>
#include <Application/LogsFilter.h>

namespace ros {

    class ROS_API LogsSink : public LogsFilter {
        public:
            LogsSink();

            virtual bool Init(const PropertyTree& config);
            virtual bool IsMessageAccepted(const LogMessage& message) const;
            void AddFilter(LogsFilterPtr filter);
            void RemoveFilter(LogsFilterPtr filter);

            virtual bool SendMessage(const LogMessage& message) =0;
            virtual void FlushMessages() =0;

        protected:
            typedef std::list<LogsFilterPtr> LogsFiltersList;
            typedef LogsFiltersList::iterator LogsFiltersIter;
            typedef LogsFiltersList::const_iterator LogsFiltersConstIter;

            LogsFiltersList filters;

        private:
            typedef Factory<std::string, LogsFilter> LogsFiltersFactory;

            LogsFiltersFactory filtersFactory;
    };

    typedef boost::shared_ptr<LogsSink> LogsSinkPtr;

}

#endif // ROS_LOGS_SINK_H

