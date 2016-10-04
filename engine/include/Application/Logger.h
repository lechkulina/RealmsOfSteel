/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGGER_H
#define ROS_LOGGER_H

#include <string>
#include <list>
#include <Core/Environment.h>
#include <Core/Factory.h>
#include <Application/LogsSink.h>

namespace ros {

    class ROS_API Logger : public LogsSink {
        public:
            Logger();

            virtual bool Init(const PropertyTree& config);

            virtual bool SendMessage(const LogMessage& message);
            virtual void FlushMessages();

        protected:
            typedef std::list<LogsSinkPtr> LogsSinksList;
            typedef LogsSinksList::iterator LogsSinksIter;
            typedef LogsSinksList::const_iterator LogsSinksConstIter;

            LogsSinksList sinks;

        private:
            typedef Factory<std::string, LogsSink> LogsSinksFactory;

            LogsSinksFactory sinksFactory;
    };

    typedef boost::shared_ptr<Logger> LoggerPtr;

}

#endif // ROS_LOGGER_H
