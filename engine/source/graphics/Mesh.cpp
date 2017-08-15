/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Mesh.h>

ros::Mesh::Mesh()
    : verticesProps(VERTICES_PROPS_NONE) {
}

void ros::Mesh::setName(const std::string& name) {
    this->name = name;
}

void ros::Mesh::addVertex(const Vertex& vertex) {
    vertices.push_back(vertex);
}

void ros::Mesh::setVerticesProps(U32 verticesProps) {
    this->verticesProps |= verticesProps;
}

void ros::Mesh::addIndices(U32 index0, U32 index1, U32 index2) {
    indices.push_back(index0);
    indices.push_back(index1);
    indices.push_back(index2);
}

void ros::Mesh::setMaterial(MaterialPtr material) {
    this->material = material;
}
