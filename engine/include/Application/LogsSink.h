/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_SINK_H
#define ROS_LOGS_SINK_H

#include <list>
#include <Core/PropertyTree.h>
#include <Core/Environment.h>
#include <Core/Factory.h>
#include <Application/LogMessage.h>
#include <Application/LogsFilter.h>

namespace ros {

    class LogsSink;
    typedef boost::shared_ptr<LogsSink> LogsSinkPtr;
    typedef Factory<String, LogsSink> LogsSinkFactory;

    class ROS_API LogsSink : public LogsFilter {
        public:
            static LogsSinkPtr Create(const PropertyTree& config);

            virtual bool Init(const PropertyTree& config);
            virtual void Uninit();
            virtual bool IsMessageAccepted(const LogMessage& message) const;

            virtual bool SendMessage(const LogMessage& message) =0;
            virtual void FlushMessages() =0;

        private:
            typedef std::list<LogsFilterPtr> LogsFiltersList;
            typedef LogsFiltersList::iterator LogsFiltersIter;
            typedef LogsFiltersList::const_iterator LogsFiltersConstIter;

            static LogsSinkFactory factory;
            LogsFiltersList filters;
    };

}

#endif // ROS_LOGS_SINK_H

