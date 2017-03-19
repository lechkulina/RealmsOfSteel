/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/SceneNode.h>

ros::SceneNode::SceneNode(const std::string& name, SceneNodePtr parent /*= SceneNodePtr()*/)
    : name(name)
    , parent(parent) {
}

void ros::SceneNode::addChild(SceneNodePtr child) {
    children.push_back(child);
}

void ros::SceneNode::setTransformation(const glm::mat4& transformation) {
    this->transformation = transformation;
}

void ros::SceneNode::addMesh(MeshPtr mesh) {
    meshes.push_back(mesh);
}
