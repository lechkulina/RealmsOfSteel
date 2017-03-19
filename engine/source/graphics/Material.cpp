/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Material.h>

ros::Material::Material(const std::string& name)
    : name(name) {
}

void ros::Material::setDiffuseColor(const glm::vec3& diffuseColor) {
    this->diffuseColor = diffuseColor;
}

void ros::Material::setSpecularColor(const glm::vec3& specularColor) {
    this->specularColor = specularColor;
}

void ros::Material::setAmbientColor(const glm::vec3& ambientColor) {
    this->ambientColor = ambientColor;
}

void ros::Material::setEmissiveColor(const glm::vec3& emissiveColor) {
    this->emissiveColor = emissiveColor;
}

void ros::Material::setTwoSided(bool twoSided) {
    this->twoSided = twoSided;
}

void ros::Material::setOpacity(float opacity) {
    this->opacity = opacity;
}

void ros::Material::setShininess(float shininess) {
    this->shininess = shininess;
}
