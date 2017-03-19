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
    enum FaceMode {
        FaceMode_Point,
        FaceMode_Line,
        FaceMode_Triangle,
        FaceMode_Polygon
    };

    class ROS_API Mesh {
        public:
            Mesh(const std::string& name, FaceMode faceMode);

            const std::string& getName() const { return name; }
            FaceMode getFaceMode() const { return faceMode; }

            const VertexVector& getVertices() const { return vertices; }
            void addVertex(const Vertex& vertex);

            const IndexVector& getIndices() const { return indices; }
            void addIndex(U32 index);

            MaterialPtr getMaterial() const { return material; }
            void setMaterial(MaterialPtr material);

        private:
            std::string name;
            FaceMode faceMode;
            VertexVector vertices;
            IndexVector indices;
            MaterialPtr material;
    };

    typedef boost::shared_ptr<Mesh> MeshPtr;
    typedef std::list<MeshPtr> MeshList;
}

#endif // ROS_MESH
