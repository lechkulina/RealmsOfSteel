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

namespace ros {
    class ROS_API Material {
        public:
            Material(const std::string& name);

            const std::string& getName() const { return name; }

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

            float getOpacity() const { return opacity; }
            void setOpacity(float opacity);

            float getShininess() const { return shininess; }
            void setShininess(float shininess);

        private:
            std::string name;
            glm::vec3 diffuseColor;
            glm::vec3 specularColor;
            glm::vec3 ambientColor;
            glm::vec3 emissiveColor;
            bool twoSided;
            float opacity;
            float shininess;
    };

    typedef boost::shared_ptr<Material> MaterialPtr;
    typedef std::map<std::string, MaterialPtr> MaterialMap;
}

#endif // ROS_MATERIAL_H

