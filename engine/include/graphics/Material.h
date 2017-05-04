/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_MATERIAL_H
#define ROS_MATERIAL_H

#include <glm/vec3.hpp>
#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Texture.h>

namespace ros {
    enum ShadingModel {
        ShadingModel_Flat,
        ShadingModel_Gouraud,
        ShadingModel_Phong,
        ShadingModel_Blinn
    };

    enum BlendMode {
        BlendMode_Default,
        BlendMode_Additive
    };

    class ROS_API Material {
        public:
            Material();

            const std::string& getName() const { return name; }
            void setName(const std::string& name);

            const glm::vec3& getDiffuseColor() const { return diffuseColor; }
            void setDiffuseColor(const glm::vec3& diffuseColor);

            const glm::vec3& getSpecularColor() const { return specularColor; }
            void setSpecularColor(const glm::vec3& specularColor);

            const glm::vec3& getAmbientColor() const { return ambientColor; }
            void setAmbientColor(const glm::vec3& ambientColor);

            const glm::vec3& getEmissiveColor() const { return emissiveColor; }
            void setEmissiveColor(const glm::vec3& emissiveColor);

            bool isTwoSided() const { return twoSided; }
            void setTwoSided(bool twoSided);

            ShadingModel getShadingModel() const { return shadingModel; }
            void setShadingModel(ShadingModel shadingModel);

            BlendMode getBlendMode() const { return blendMode; }
            void setBlendMode(BlendMode blendMode);

            float getOpacity() const { return opacity; }
            void setOpacity(float opacity);

            float getShininess() const { return shininess; }
            void setShininess(float shininess);

            float getShininessStrength() const { return shininessStrength; }
            void setShininessStrength(float shininessStrength);

            const TexturesStack& getDiffuseTextures() const { return diffuseTextures; }
            void addDiffuseTexture(const Texture& texture);

            const TexturesStack& getSpecularTextures() const { return specularTextures; }
            void addSpecularTexture(const Texture& texture);

            const TexturesStack& getAmbientTextures() const { return ambientTextures; }
            void addAmbientTexture(const Texture& texture);

            const TexturesStack& getEmissiveTextures() const { return emissiveTextures; }
            void addEmissiveTexture(const Texture& texture);

            const TexturesStack& getGlossinessMaps() const { return glossinessMaps; }
            void addGlossinessMap(const Texture& map);

            const TexturesStack& getHeightMaps() const { return heightMaps; }
            void addHeightMap(const Texture& map);

            const TexturesStack& getNormalMaps() const { return normalMaps; }
            void addNormalMap(const Texture& map);

            const TexturesStack& getLightMaps() const { return lightMaps; }
            void addLightMap(const Texture& map);

        private:
            std::string name;
            glm::vec3 diffuseColor;
            glm::vec3 specularColor;
            glm::vec3 ambientColor;
            glm::vec3 emissiveColor;
            ShadingModel shadingModel;
            BlendMode blendMode;
            bool twoSided;
            float opacity;
            float shininess;
            float shininessStrength;
            TexturesStack diffuseTextures;
            TexturesStack specularTextures;
            TexturesStack ambientTextures;
            TexturesStack emissiveTextures;
            TexturesStack glossinessMaps;
            TexturesStack heightMaps;
            TexturesStack normalMaps;
            TexturesStack lightMaps;
    };

    typedef boost::shared_ptr<Material> MaterialPtr;
    typedef std::vector<MaterialPtr> MaterialsVector;
    typedef std::map<std::string, MaterialPtr> MaterialsMap;
}

#endif // ROS_MATERIAL_H

