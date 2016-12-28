/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SHADERS_MANAGER_H
#define ROS_SHADERS_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Shader.h>

namespace ros {
    class ROS_API ShadersManager: public boost::noncopyable {
        public:
            static ShadersManager* getInstance();

            bool prepare(const PropertyTree& config);
            ShaderPtr provide(const PropertyTree& config);
            void clear();

        private:
            ShaderMap shaders;
    };
}

#endif // ROS_SHADERS_MANAGER_H

