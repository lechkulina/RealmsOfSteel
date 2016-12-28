/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_PROGRAMS_MANAGER_H
#define ROS_PROGRAMS_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Program.h>

namespace ros {
    class ROS_API ProgramsManager: public boost::noncopyable {
        public:
            static ProgramsManager* getInstance();

            bool prepare(const PropertyTree& config);
            ProgramPtr provide(const PropertyTree& config);
            void clear();

        private:
            ProgramMap programs;
    };
}

#endif // ROS_PROGRAMS_MANAGER_H

