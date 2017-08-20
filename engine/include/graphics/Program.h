/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_PROGRAM_H
#define ROS_PROGRAM_H

#include <glm/glm.hpp>
#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Shader.h>

namespace ros {
    class Program;
    typedef boost::shared_ptr<Program> ProgramPtr;
    typedef std::list<ProgramPtr> ProgramList;
    typedef std::map<std::string, ProgramPtr> ProgramMap;

    class ROS_API Program: public boost::noncopyable {
        public:
            virtual ~Program() {}

            static ProgramPtr make();

            virtual bool create() =0;
            virtual bool attachShader(ShaderPtr shader) =0;
            virtual bool link() =0;
            virtual void free() =0;
            virtual bool isLinked() const =0;

            virtual bool bind() =0;
            virtual void unbind() =0;

            virtual bool setUniform(const char* name, int value) =0;
            virtual bool setUniform(const char* name, const glm::vec4& value) =0;
            virtual bool setUniform(const char* name, const glm::mat4& value) =0;
    };
}

#endif // ROS_PROGRAM_H

