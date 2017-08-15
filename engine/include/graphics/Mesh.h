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

    class ROS_API Mesh {
        public:
            static const U32 VERTICES_PROPS_NONE = 0;
            static const U32 VERTICES_PROPS_NORMALS = 1 << 1;
            static const U32 VERTICES_PROPS_TENGENTS = 1 << 2;
            static const U32 VERTICES_PROPS_BITANGENTS = 1 << 3;
            static const U32 VERTICES_PROPS_COLORS = 1 << 4;
            static const U32 VERTICES_PROPS_TEXTURE_COORDS = 1 << 5;

            Mesh();

            void setName(const std::string& name);
            const std::string& getName() const { return name; }
            void addVertex(const Vertex& vertex);
            const VerticesVector& getVertices() const { return vertices; }
            void setVerticesProps(U32 verticesProps);
            U32 getVerticesProps() const { return verticesProps; }
            void addIndices(U32 index0, U32 index1, U32 index2);
            const IndicesVector& getIndices() const { return indices; }
            void setMaterial(MaterialPtr material);
            MaterialPtr getMaterial() const { return material; }

        private:
            std::string name;
            VerticesVector vertices;
            U32 verticesProps;
            IndicesVector indices;
            MaterialPtr material;
    };

    typedef boost::shared_ptr<Mesh> MeshPtr;
    typedef std::vector<MeshPtr> MeshesVector;
}

#endif // ROS_MESH
