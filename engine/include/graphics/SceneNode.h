/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SCENE_NODE_H
#define ROS_SCENE_NODE_H

#include <glm/mat4x4.hpp>
#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Mesh.h>

namespace ros {
    class SceneNode;
    typedef boost::shared_ptr<SceneNode> SceneNodePtr;
    typedef boost::weak_ptr<SceneNode> SceneNodeWeakPtr;
    typedef std::list<SceneNodePtr> SceneNodesList;

    class ROS_API SceneNode {
        public:
            void setName(const std::string& name);
            const std::string& getName() const { return name; }

            void setParent(SceneNodeWeakPtr parent);
            SceneNodeWeakPtr getParent() const { return parent; }
            bool isRoot() const { return parent.expired(); }
            bool isLeaf() const { return children.empty(); }

            void addChild(SceneNodePtr child);
            const SceneNodesList& getChildren() const { return children; }

            void setTransformation(const glm::mat4& transformation);
            const glm::mat4& getTransformation() const { return transformation; }

            void addMesh(MeshPtr mesh);
            const MeshesVector& getMeshes() const { return meshes; }

        private:
            std::string name;
            SceneNodeWeakPtr parent;
            SceneNodesList children;
            glm::mat4 transformation;
            MeshesVector meshes;
    };
}

#endif // ROS_SCENE_NODE_H

