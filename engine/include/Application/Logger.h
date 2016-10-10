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

            inline bool SendTrace(const LogFormat& format) {
                return SendMessage(LogMessage(LogLevel_Trace, format.str()));
            }

            inline bool SendDebug(const LogFormat& format) {
                return SendMessage(LogMessage(LogLevel_Debug, format.str()));
            }

            inline bool SendWarning(const LogFormat& format) {
                return SendMessage(LogMessage(LogLevel_Warning, format.str()));
            }

            inline bool SendError(const LogFormat& format) {
                return SendMessage(LogMessage(LogLevel_Error, format.str()));
            }

            inline bool SendCritical(const LogFormat& format) {
                return SendMessage(LogMessage(LogLevel_Critical, format.str()));
            }

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

