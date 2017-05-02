/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <application/Logger.h>
#include "AssimpIOSystem.h"
#include "AssimpSceneLoader.h"

ros::AssimpSceneLoader::AssimpSceneLoader()
    : importer(ROS_NULL) {
    importer = new Assimp::Importer;
    importer->SetIOHandler(new AssimpIOSystem());
}

ros::AssimpSceneLoader::~AssimpSceneLoader() {
    delete importer;
}

bool ros::AssimpSceneLoader::isLoadable(const std::string& name) const {
    if (name.length() <= MAX_EXTENSION_LENGTH) {
        return false;
    }
    const std::string extension = name.substr(name.length() - MAX_EXTENSION_LENGTH);
    return importer->IsExtensionSupported(extension);
}

bool ros::AssimpSceneLoader::extractMeshes(const aiScene* srcScene, MeshesList& dstMeshes) {
    for (unsigned int meshIdx=0; meshIdx < srcScene->mNumMeshes; ++meshIdx) {
        const aiMesh* const srcMesh = srcScene->mMeshes[meshIdx];
        if (!srcMesh->HasPositions()) {
            Logger::report(LogLevel_Error, boost::format("Mesh %s does not contain positions") % srcMesh->mName.C_Str());
            return false;
        }
        if (!srcMesh->HasFaces()) {
            Logger::report(LogLevel_Error, boost::format("Mesh %s does not contain faces") % srcMesh->mName.C_Str());
            return false;
        }

        MeshPtr dstMesh = boost::make_shared<Mesh>();
        dstMesh->setName(srcMesh->mName.C_Str());
        unsigned int indicesPerFace = 0;
        switch(srcMesh->mPrimitiveTypes) {  // support only a single non-polygon faces type per mesh
            case aiPrimitiveType_POINT:
                dstMesh->setFacesType(FaceType_Point);
                indicesPerFace = 1;
                break;
            case aiPrimitiveType_LINE:
                dstMesh->setFacesType(FaceType_Line);
                indicesPerFace = 2;
                break;
            case aiPrimitiveType_TRIANGLE:
                dstMesh->setFacesType(FaceType_Triangle);
                indicesPerFace = 3;
                break;
            default:
                Logger::report(LogLevel_Error, boost::format("Unsupported faces type found in mesh %s") % srcMesh->mName.C_Str());
                return false;
        }

        dstMesh->setNormals(srcMesh->HasNormals());
        dstMesh->setTangents(srcMesh->HasTangentsAndBitangents());
        dstMesh->setBitangents(srcMesh->HasTangentsAndBitangents());

        unsigned int colorsPerVertex = srcMesh->GetNumColorChannels();
        if (colorsPerVertex > Vertex::MAX_COLORS) {
            colorsPerVertex = Vertex::MAX_COLORS;
            Logger::report(LogLevel_Warning, boost::format("Mesh %s has %d colors per vertex but only %d are supported")
                                % srcMesh->mName.C_Str() % srcMesh->GetNumColorChannels() % (int)Vertex::MAX_COLORS);
        }
        dstMesh->setColorsPerVertex(colorsPerVertex);

        unsigned int textureCoordsPerVertex = srcMesh->GetNumUVChannels();
        if (textureCoordsPerVertex > Vertex::MAX_TEXTURE_COORDS) {
            textureCoordsPerVertex = Vertex::MAX_TEXTURE_COORDS;
            Logger::report(LogLevel_Warning, boost::format("Mesh %s has %d texture coords per vertex but only %d are supported")
                                % srcMesh->mName.C_Str() % srcMesh->GetNumUVChannels() % (int)Vertex::MAX_TEXTURE_COORDS);
        }
        dstMesh->setTextureCoordsPerVertex(textureCoordsPerVertex);
        unsigned int textureCoordsComponents = 0;  // support only the same number of texture coords components for all the texture coords
        if (textureCoordsPerVertex > 0) {
            textureCoordsComponents = srcMesh->mNumUVComponents[0];
            for (unsigned int textureCoordsIdx=1; textureCoordsIdx < textureCoordsPerVertex; ++textureCoordsIdx) {
                if (srcMesh->mNumUVComponents[textureCoordsIdx] != textureCoordsComponents) {
                    Logger::report(LogLevel_Error, boost::format("Mixed number number of %d (instead %d) texture coords components found in mesh %s")
                                        % srcMesh->mNumUVComponents[textureCoordsIdx] % textureCoordsComponents % srcMesh->mName.C_Str());
                    return false;
                }
            }
        }
        dstMesh->setTextureCoordsComponents(textureCoordsComponents);

        for (unsigned int vertexIdx=0; vertexIdx < srcMesh->mNumVertices; ++vertexIdx) {
            Vertex dstVertex;
            const aiVector3D& srcPosition = srcMesh->mVertices[vertexIdx];
            dstVertex.position.x = srcPosition.x;
            dstVertex.position.y = srcPosition.y;
            dstVertex.position.z = srcPosition.z;

            if (srcMesh->HasNormals()) {
                const aiVector3D& srcNormal = srcMesh->mNormals[vertexIdx];
                dstVertex.normal.x = srcNormal.x;
                dstVertex.normal.y = srcNormal.y;
                dstVertex.normal.z = srcNormal.z;
            }

            if (srcMesh->HasTangentsAndBitangents()) {
                const aiVector3D& srcTangent = srcMesh->mTangents[vertexIdx];
                dstVertex.tangent.x = srcTangent.x;
                dstVertex.tangent.y = srcTangent.y;
                dstVertex.tangent.z = srcTangent.z;

                const aiVector3D& srcBitangent = srcMesh->mBitangents[vertexIdx];
                dstVertex.bitangent.x = srcBitangent.x;
                dstVertex.bitangent.y = srcBitangent.y;
                dstVertex.bitangent.z = srcBitangent.z;
            }

            for (unsigned int colorIdx=0; colorIdx < colorsPerVertex; ++colorIdx) {
                const aiColor4D& srcColor = srcMesh->mColors[colorIdx][vertexIdx];
                glm::vec4& dstColor = dstVertex.colors[colorIdx];
                dstColor.r = srcColor.r;
                dstColor.g = srcColor.g;
                dstColor.b = srcColor.b;
                dstColor.a = srcColor.a;
            }

            for (unsigned int coordsIdx=0; coordsIdx < textureCoordsPerVertex; ++coordsIdx) {
                const aiVector3D& srcTextureCoords = srcMesh->mTextureCoords[coordsIdx][vertexIdx];
                glm::vec3& dstTextureCoords = dstVertex.textureCoords[coordsIdx];
                for (unsigned int componentIdx=0; componentIdx < textureCoordsComponents; ++componentIdx) {
                    dstTextureCoords[componentIdx] = srcTextureCoords[componentIdx];
                }
            }

            dstMesh->addVertex(dstVertex);
        }

        for (unsigned int faceIdx=0; faceIdx < srcMesh->mNumFaces; ++faceIdx) {
            const aiFace& srcFace = srcMesh->mFaces[faceIdx];
            if (srcFace.mNumIndices != indicesPerFace) {
                Logger::report(LogLevel_Error, boost::format("Unsupported number of indices %d (instead %d) found in mesh %s")
                                % srcFace.mNumIndices % indicesPerFace % srcMesh->mName.C_Str());
                return false;
            }
            for (unsigned int indexIdx=0; indexIdx < indicesPerFace; ++indexIdx) {
                dstMesh->addIndex(srcFace.mIndices[indexIdx]);
            }
        }

        dstMeshes.push_back(dstMesh);
    }

    return true;
}

ros::ResourcePtr ros::AssimpSceneLoader::loadResource(const std::string& name) {
    const aiScene* const srcScene = importer->ReadFile(name.c_str(), aiProcess_Triangulate|aiProcess_SortByPType);
    if (!srcScene) {
        Logger::report(LogLevel_Error, boost::format("Failed to load scene for resource %s - Assimp error occured %s")
                            % name % importer->GetErrorString());
        return ScenePtr();
    }

    MeshesList dstMeshes;
    if (!extractMeshes(srcScene, dstMeshes)) {
        Logger::report(LogLevel_Error, boost::format("Failed to create scene for resource %s") % name);
        importer->FreeScene();
        return ScenePtr();
    }

    importer->FreeScene();
    return ScenePtr();
}
