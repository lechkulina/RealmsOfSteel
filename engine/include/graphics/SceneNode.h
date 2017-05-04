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
    typedef std::list<SceneNodePtr> SceneNodeList;

    class ROS_API SceneNode {
        public:
            explicit SceneNode(const std::string& name, SceneNodePtr parent = SceneNodePtr());

            const std::string& getName() const { return name; }
            SceneNodeWeakPtr getParent() const { return parent; }
            bool isRoot() const { return parent.expired(); }
            bool isLeaf() const { return children.empty(); }

            const SceneNodeList& getChildren() const;
            void addChild(SceneNodePtr child);

            const glm::mat4& getTransformation() const { return transformation; }
            void setTransformation(const glm::mat4& transformation);

            const MeshesVector& getMeshes() const { return meshes; }
            void addMesh(MeshPtr mesh);

        private:
            std::string name;
            SceneNodeWeakPtr parent;
            SceneNodeList children;
            glm::mat4 transformation;
            MeshesVector meshes;
    };
}

#endif // ROS_SCENE_NODE_H

