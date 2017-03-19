/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SCENE_H
#define ROS_SCENE_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Buffer.h>
#include <graphics/SceneNode.h>

namespace ros {
    class ROS_API Scene: public Buffer {
        public:
            Scene(const std::string& name, SceneNodePtr root = SceneNodePtr());
            const std::string& getName() const { return name; }

            const SceneNodePtr getRoot() const { return root; }
            void setRoot(SceneNodePtr root);

            virtual bool isNull() const { return root; }
            virtual U32 getSize() const { return 0; /* TODO */ }

        private:
            std::string name;
            SceneNodePtr root;
    };

    typedef boost::shared_ptr<Scene> ScenePtr;
}

#endif // ROS_SCENE_H

