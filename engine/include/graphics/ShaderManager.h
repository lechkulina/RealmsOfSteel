/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SHADER_MANAGER_H
#define ROS_SHADER_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Shader.h>

namespace ros {
    class ROS_API ShaderManager: public boost::noncopyable {
        public:
            ~ShaderManager();

            bool init(const PropertyTree& config);
            void uninit();

            ShaderPtr initShader(const PropertyTree& config);
            bool initShaders(const PropertyTree& config, ShaderList& dst);

        private:
            ShaderMap shaders;
    };
}

#endif // ROS_SHADER_MANAGER_H

