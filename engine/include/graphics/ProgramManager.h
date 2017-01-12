/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_PROGRAM_MANAGER_H
#define ROS_PROGRAM_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Program.h>

namespace ros {
    class ROS_API ProgramManager: public boost::noncopyable {
        public:
            ~ProgramManager();

            bool init(const PropertyTree& config);
            void uninit();

            ProgramPtr initProgram(const PropertyTree& config);
            bool initPrograms(const PropertyTree& config, ProgramList& dst);

        private:
            ProgramMap programs;
    };
}

#endif // ROS_PROGRAM_MANAGER_H

