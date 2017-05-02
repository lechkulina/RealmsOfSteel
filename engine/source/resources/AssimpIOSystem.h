/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ASSIMP_IOSYSTEM_H
#define ROS_ASSIMP_IOSYSTEM_H

#include <assimp/IOSystem.hpp>

namespace ros {
    class AssimpIOSystem: public Assimp::IOSystem {
        public:
            virtual void Close(Assimp::IOStream* file);
            virtual bool Exists(const char* file) const;
            virtual char getOsSeparator() const;
            virtual Assimp::IOStream* Open(const char* file, const char* mode = "rb");
    };
}

#endif // ROS_ASSIMP_IOSYSTEM_H

