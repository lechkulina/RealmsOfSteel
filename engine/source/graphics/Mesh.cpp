/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Mesh.h>

ros::Mesh::Mesh()
    : normals(false)
    , tangents(false)
    , bitangents(false)
    , colorsPerVertex(0)
    , textureCoordsPerVertex(0)
    , textureCoordsComponents(0)
    , facesType(FaceType_None) {
}

void ros::Mesh::setName(const std::string& name) {
    this->name = name;
}

void ros::Mesh::addVertex(const Vertex& vertex) {
    vertices.push_back(vertex);
}

void ros::Mesh::setNormals(bool normals) {
    this->normals = normals;
}

void ros::Mesh::setTangents(bool tangents) {
    this->tangents = tangents;
}

void ros::Mesh::setBitangents(bool bitangents) {
    this->bitangents = bitangents;
}

void ros::Mesh::setColorsPerVertex(U32 colorsPerVertex) {
    this->colorsPerVertex = colorsPerVertex > Vertex::MAX_COLORS ? Vertex::MAX_COLORS : colorsPerVertex;
}

void ros::Mesh::setTextureCoordsPerVertex(U32 textureCoordsPerVertex) {
    this->textureCoordsPerVertex = textureCoordsPerVertex > Vertex::MAX_TEXTURE_COORDS ? Vertex::MAX_TEXTURE_COORDS : textureCoordsPerVertex;
}

void ros::Mesh::setTextureCoordsComponents(U32 textureCoordsComponents) {
    this->textureCoordsComponents = std::min(textureCoordsComponents, 3U);
}

void ros::Mesh::addIndex(U32 index) {
    indices.push_back(index);
}

void ros::Mesh::setFacesType(FaceType facesType) {
    this->facesType = facesType;
}

ros::U32 ros::Mesh::getIndicesPerFace() const {
    switch(facesType) {
        case FaceType_Point:
            return 1;
        case FaceType_Line:
            return 2;
        case FaceType_Triangle:
            return 3;
        default:
            return 0;
    }
}

void ros::Mesh::setMaterial(MaterialPtr material) {
    this->material = material;
}
