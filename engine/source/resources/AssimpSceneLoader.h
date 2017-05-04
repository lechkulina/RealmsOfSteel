/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ASSIMP_SCENE_LOADER_H
#define ROS_ASSIMP_SCENE_LOADER_H

#include <assimp/Importer.hpp>
#include <core/Common.h>
#include <resources/ResourceLoader.h>
#include <graphics/Scene.h>

namespace ros {
    class AssimpSceneLoader: public ResourceLoader {
        public:
            AssimpSceneLoader();
            virtual ~AssimpSceneLoader();

            virtual bool isLoadable(const std::string& name) const;
            virtual ResourcePtr loadResource(const std::string& name);

        private:
            static const size_t MAX_EXTENSION_LENGTH = 4;

            Assimp::Importer* importer;

            bool extractMaterials(const aiScene* srcScene, MaterialsVector& dstMaterials);
            bool extractMeshes(const aiScene* srcScene, MeshesVector& dstMeshes);
    };
}

#endif // ROS_ASSIMP_SCENE_LOADER_H

