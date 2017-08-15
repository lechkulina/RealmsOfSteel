/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_TEXTURE_H
#define ROS_TEXTURE_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    class ROS_API Texture {
        public:
            const std::string& getName() const { return name; }
            void setName(const std::string& name);

        private:
            std::string name;
    };
}

#endif // ROS_TEXTURE_H

