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
    class ROS_API Shader: public boost::noncopyable {
        public:
            virtual ~Shader() {}

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual bool isValid() const =0;

            virtual const std::string& getPath() const =0;

            const std::string& getName() const { return name; }

        private:
            std::string name;
    };

    typedef boost::shared_ptr<Shader> ShaderPtr;
    typedef std::list<ShaderPtr> ShaderList;
    typedef std::map<std::string, ShaderPtr> ShaderMap;
}

#endif // ROS_SHADER_H

