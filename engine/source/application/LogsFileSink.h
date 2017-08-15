/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_LOGS_FILE_SINK_H
#define ROS_LOGS_FILE_SINK_H

#include <fstream>
#include <application/LogsSink.h>

namespace ros {
    typedef boost::optional<std::ios_base::openmode> OpenModeOpt;

    class LogsFileSink : public LogsSink {
        public:
            static const fs::path DEFAULT_PATH;
            static const std::string DEFAULT_OPEN_MODE;

            virtual ~LogsFileSink();

            virtual bool open(const fs::path& path, std::ios_base::openmode openMode);
            virtual void close();
            bool isOpen() const { return stream.is_open(); }
            const fs::path& getPath() const { return path; }
            OpenModeOpt getOpenMode() const { return openMode; }

            virtual bool init(const pt::ptree& config);

            virtual bool sendEntry(const LogEntry& entry);
            virtual void flushEntries();

        private:
            std::ofstream stream;
            fs::path path;
            OpenModeOpt openMode;
    };
}

#endif // ROS_LOGS_FILE_SINK_H

