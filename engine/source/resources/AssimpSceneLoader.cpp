/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
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

ros::MaterialPtr ros::AssimpSceneLoader::createMaterial(const aiMaterial* src) {
    aiString name;
    if (src->Get(AI_MATKEY_NAME, name) != AI_SUCCESS) {
        ROS_ERROR(boost::format("Failed to get material name"));
        return MaterialPtr();
    }

    MaterialPtr material = boost::make_shared<Material>();
    material->setName(name.C_Str());

    aiColor3D diffuseColor;
    if (src->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) == AI_SUCCESS) {
        material->setDiffuseColor(glm::vec3(diffuseColor.r, diffuseColor.g, diffuseColor.b));
    } else {
        ROS_WARNING(boost::format("Failed to get diffuse color form material %s") % name.C_Str());
    }

    aiColor3D specularColor;
    if (src->Get(AI_MATKEY_COLOR_SPECULAR, specularColor) == AI_SUCCESS) {
        material->setSpecularColor(glm::vec3(specularColor.r, specularColor.g, specularColor.b));
    } else {
        ROS_WARNING(boost::format("Failed to get specular color form material %s") % name.C_Str());
    }

    aiColor3D ambientColor;
    if (src->Get(AI_MATKEY_COLOR_AMBIENT, ambientColor) == AI_SUCCESS) {
        material->setAmbientColor(glm::vec3(ambientColor.r, ambientColor.g, ambientColor.b));
    } else {
        ROS_WARNING(boost::format("Failed to get ambient color form material %s") % name.C_Str());
    }

    aiColor3D emissiveColor;
    if (src->Get(AI_MATKEY_COLOR_EMISSIVE, emissiveColor) == AI_SUCCESS) {
        material->setEmissiveColor(glm::vec3(emissiveColor.r, emissiveColor.g, emissiveColor.b));
    } else {
        ROS_WARNING(boost::format("Failed to get emissive color form material %s") % name.C_Str());
    }

    int twoSided = 0;
    if (src->Get(AI_MATKEY_TWOSIDED, twoSided) == AI_SUCCESS) {
        material->setTwoSided(twoSided != 0);
    } else {
        ROS_WARNING(boost::format("Failed to get sides info form material %s") % name.C_Str());
    }

    float opacity = 0.0f;
    if (src->Get(AI_MATKEY_OPACITY, opacity) == AI_SUCCESS) {
        material->setOpacity(opacity);
    } else {
        ROS_WARNING(boost::format("Failed to get opacity form material %s") % name.C_Str());
    }

    float shininess = 0.0f;
    if (src->Get(AI_MATKEY_SHININESS, shininess) == AI_SUCCESS) {
        material->setShininess(shininess);
    } else {
        ROS_WARNING(boost::format("Failed to get shininess form material %s") % name.C_Str());
    }

    return material;
}

ros::MaterialsVector ros::AssimpSceneLoader::createMaterials(const aiScene* src) {
    MaterialsVector materials;
    if (!src->HasMaterials()) {
        return materials;
    }
    for (unsigned int i=0; i < src->mNumMaterials; ++i) {
        MaterialPtr material = createMaterial(src->mMaterials[i]);
        if (!material) {
            materials.clear();
            return materials;
        }
        materials.push_back(material);
    }
    return materials;
}

ros::MeshPtr ros::AssimpSceneLoader::createMesh(const aiMesh* src, const MaterialsVector& materials) {
    const aiString& name = src->mName;
    if (!src->HasPositions() || !src->HasFaces()) {
        ROS_ERROR(boost::format("Mesh %s is missing faces data") % name.C_Str());
        return MeshPtr();
    }
    if (src->mPrimitiveTypes != aiPrimitiveType_TRIANGLE) {
        ROS_ERROR(boost::format("Mesh %s uses non-triangular primitive type") % name.C_Str());
        return MeshPtr();
    }

    MeshPtr mesh = boost::make_shared<Mesh>();
    mesh->setName(name.C_Str());

    bool hasNormals = src->HasNormals();
    if (hasNormals) {
        mesh->setVerticesProps(Mesh::VERTICES_PROPS_NORMALS);
    }
    bool hasTangents = src->HasTangentsAndBitangents();
    if (hasTangents) {
        mesh->setVerticesProps(Mesh::VERTICES_PROPS_TENGENTS);
        mesh->setVerticesProps(Mesh::VERTICES_PROPS_BITANGENTS);
    }
    unsigned int colorChannels = src->GetNumColorChannels();
    if (colorChannels > 0) {
        mesh->setVerticesProps(Mesh::VERTICES_PROPS_COLORS);
        if (colorChannels > 1) {
            ROS_WARNING(boost::format("Mesh %s has %d colors per vertex but only 1 is supported") % name.C_Str() % colorChannels);
        }
    }
    unsigned int uvChannels = src->GetNumUVChannels();
    if (uvChannels > 0) {
        mesh->setVerticesProps(Mesh::VERTICES_PROPS_TEXTURE_COORDS);
        if (uvChannels > 1) {
            ROS_WARNING(boost::format("Mesh %s has %d texture coords per vertex but only 1 is supported") % name.C_Str() % uvChannels);
        }
    }
    for (unsigned int j=1; j < uvChannels; ++j) {
        if (src->mNumUVComponents[j] != 2) {
            ROS_WARNING(boost::format("Mesh %s uses %d texture coords components but only 2 are supported") % name.C_Str() % src->mNumUVComponents[j]);
        }
    }

    for (unsigned int j=0; j < src->mNumVertices; ++j) {
        Vertex vertex;
        const aiVector3D& position = src->mVertices[j];
        vertex.position = glm::vec3(position.x, position.y, position.z);
        if (hasNormals) {
            const aiVector3D& normal = src->mNormals[j];
            vertex.normal = glm::vec3(normal.x, normal.y, normal.z);
        }
        if (hasTangents) {
            const aiVector3D& tangent = src->mTangents[j];
            const aiVector3D& bitangent = src->mBitangents[j];
            vertex.tangent = glm::vec3(tangent.x, tangent.y, tangent.z);
            vertex.bitangent = glm::vec3(bitangent.x, bitangent.y, bitangent.z);
        }
        if (colorChannels > 0) {
            const aiColor4D& color = src->mColors[0][j];
            vertex.color = glm::vec4(color.r, color.g, color.b, color.a);
        }
        if (uvChannels > 0) {
            const aiVector3D& textureCoords = src->mTextureCoords[0][j];
            vertex.textureCoords = glm::vec2(textureCoords.x, textureCoords.y);
        }
        mesh->addVertex(vertex);
    }

    for (unsigned int j=0; j < src->mNumFaces; ++j) {
        const aiFace& face = src->mFaces[j];
        if (face.mNumIndices != 3) {
            ROS_ERROR(boost::format("Mesh %s uses unsupported number of indices %d") % name.C_Str() % face.mNumIndices);
            return MeshPtr();
        }
        mesh->addIndices(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
    }

    unsigned int materialIndex = src->mMaterialIndex;
    if (materialIndex >= materials.size()) {
        ROS_ERROR(boost::format("Mesh %s uses invalid material index %d - number of found materials is %d)")
                    % name.C_Str() % materialIndex % materials.size());
        return MeshPtr();
    }
    mesh->setMaterial(materials[materialIndex]);

    return mesh;
}

ros::MeshesVector ros::AssimpSceneLoader::createMeshes(const aiScene* src, const MaterialsVector& materials) {
    MeshesVector meshes;
    if (!src->HasMeshes()) {
        return meshes;
    }
    for (unsigned int i=0; i < src->mNumMeshes; ++i) {
        MeshPtr mesh = createMesh(src->mMeshes[i], materials);
        if (!mesh) {
            meshes.clear();
            return meshes;
        }
        meshes.push_back(mesh);
    }
    return meshes;
}

ros::SceneNodePtr ros::AssimpSceneLoader::createSceneNode(const aiNode* src, const MeshesVector& meshes, SceneNodeWeakPtr parent /*= SceneNodeWeakPtr()*/) {
    const aiString& name = src->mName;

    SceneNodePtr sceneNode = boost::make_shared<SceneNode>();
    sceneNode->setName(name.C_Str());
    sceneNode->setParent(parent);

    for (unsigned int i=0; i < src->mNumMeshes; ++i) {
        unsigned int meshIndex = src->mMeshes[i];
        if (meshIndex >= meshes.size()) {
            ROS_ERROR(boost::format("Scene node %s uses invalid mesh index %d - number of found meshes is %d")
                        % name.C_Str() % meshIndex % meshes.size());
            return SceneNodePtr();
        }
        sceneNode->addMesh(meshes[meshIndex]);
    }

    const aiMatrix4x4& trans = src->mTransformation;
    sceneNode->setTransformation(glm::mat4x4(
        trans[0][0], trans[0][1], trans[0][2], trans[0][3],
        trans[1][0], trans[1][1], trans[1][2], trans[1][3],
        trans[2][0], trans[2][1], trans[2][2], trans[2][3],
        trans[3][0], trans[3][1], trans[3][2], trans[3][3]
    ));

    for (unsigned int i=0; i < src->mNumChildren; ++i) {
        SceneNodePtr child = createSceneNode(src->mChildren[i], meshes, sceneNode);
        if (!child) {
            return SceneNodePtr();
        }
        sceneNode->addChild(child);
    }

    return sceneNode;
}

ros::ScenePtr ros::AssimpSceneLoader::createScene(const aiScene* src) {
    MaterialsVector materials = createMaterials(src);
    MeshesVector meshes = createMeshes(src, materials);
    if (meshes.empty()) {
        return ScenePtr();
    }

    SceneNodePtr root = createSceneNode(src->mRootNode, meshes);
    if (!root) {
        return ScenePtr();
    }

    ScenePtr scene = boost::make_shared<Scene>();
    scene->setRoot(root);

    return scene;
}

ros::ResourcePtr ros::AssimpSceneLoader::loadResource(const std::string& name) {
    const aiScene* const src = importer->ReadFile(name.c_str(), aiProcess_Triangulate|aiProcess_SortByPType);
    if (!src) {
        ROS_ERROR(boost::format("Failed to load scene for resource %s - Assimp error occured %s") % name % importer->GetErrorString());
        return ScenePtr();
    }
    ScenePtr scene = createScene(src);
    importer->FreeScene();
    return scene;
}
