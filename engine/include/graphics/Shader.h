/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SHADER_H
#define ROS_SHADER_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum ShaderType {
        ShaderType_Compute,
        ShaderType_Vertex,
        ShaderType_TessControl,
        ShaderType_TessEvaluation,
        ShaderType_Geometry,
        ShaderType_Fragment
    };

    class Shader;
    typedef boost::shared_ptr<Shader> ShaderPtr;

    class ROS_API Shader: public boost::noncopyable {
        public:
            virtual ~Shader() {}

            static ShaderPtr make();

            virtual bool create(ShaderType type) =0;
            virtual bool uploadSource(const fs::path& path) =0;
            virtual bool compile() =0;
            virtual void free() =0;
            virtual bool isCompiled() const =0;
            virtual const fs::path& getPath() const =0;
    };
}

#endif // ROS_SHADER_H

