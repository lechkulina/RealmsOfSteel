/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_MESH
#define ROS_MESH

#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Vertex.h>
#include <graphics/Material.h>

namespace ros {
    typedef std::vector<U32> IndicesVector;

    enum FaceType {
        FaceType_None,
        FaceType_Point,
        FaceType_Line,
        FaceType_Triangle
    };

    class ROS_API Mesh {
        public:
            Mesh();

            void setName(const std::string& name);
            const std::string& getName() const { return name; }

            const VerticesVector& getVertices() const { return vertices; }
            void addVertex(const Vertex& vertex);

            void setNormals(bool normals);
            bool hasNormals() const { return normals; }

            void setTangents(bool tangents);
            bool hasTangents() const { return tangents; }

            void setBitangents(bool bitangents);
            bool hasBitangents() const { return bitangents; }

            bool hasColors() const { return getColorsPerVertex() > 0; }
            void setColorsPerVertex(U32 colorsPerVertex);
            U32 getColorsPerVertex() const { return colorsPerVertex; }

            void setTextureCoordsPerVertex(U32 textureCoordsPerVertex);
            U32 getTextureCoordsPerVertex() const { return textureCoordsPerVertex; }
            bool hasTextureCoords() const { return getTextureCoordsPerVertex() > 0; }
            void setTextureCoordsComponents(U32 textureCoordsComponents);
            U32 getTextureCoordsComponents() const { return textureCoordsComponents; }

            const IndicesVector& getIndices() const { return indices; }
            void addIndex(U32 index);

            void setFacesType(FaceType facesType);
            FaceType getFacesType() const { return facesType; }
            U32 getIndicesPerFace() const;

            void setMaterial(MaterialPtr material);
            MaterialPtr getMaterial() const { return material; }

        private:
            std::string name;
            VerticesVector vertices;
            bool normals;
            bool tangents;
            bool bitangents;
            U32 colorsPerVertex;
            U32 textureCoordsPerVertex;
            U32 textureCoordsComponents;
            IndicesVector indices;
            FaceType facesType;
            MaterialPtr material;
    };

    typedef boost::shared_ptr<Mesh> MeshPtr;
    typedef std::vector<MeshPtr> MeshesVector;
}

#endif // ROS_MESH
