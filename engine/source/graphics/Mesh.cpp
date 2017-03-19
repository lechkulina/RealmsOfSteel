/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Mesh.h>

ros::Mesh::Mesh(const std::string& name, FaceMode faceMode)
    : name(name)
    , faceMode(faceMode) {
}

void ros::Mesh::addVertex(const Vertex& vertex) {
    vertices.push_back(vertex);
}

void ros::Mesh::addIndex(U32 index) {
    indices.push_back(index);
}

void ros::Mesh::setMaterial(MaterialPtr material) {
    this->material = material;
}
