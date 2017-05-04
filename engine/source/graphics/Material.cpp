/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <graphics/Material.h>

ros::Material::Material()
    : shadingModel(ShadingModel_Blinn)
    , blendMode(BlendMode_Default)
    , twoSided(false)
    , opacity(0.0f)
    , shininess(0.0f)
    , shininessStrength(0.0f) {
}

void ros::Material::setName(const std::string& name) {
    this->name = name;
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

void ros::Material::setShadingModel(ShadingModel shadingModel) {
    this->shadingModel = shadingModel;
}

void ros::Material::setBlendMode(BlendMode blendMode) {
    this->blendMode = blendMode;
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

void ros::Material::setShininessStrength(float shininessStrength) {
    this->shininessStrength = shininessStrength;
}

void ros::Material::addDiffuseTexture(const Texture& texture) {
    diffuseTextures.push_back(texture);
}

void ros::Material::addSpecularTexture(const Texture& texture) {
    specularTextures.push_back(texture);
}

void ros::Material::addAmbientTexture(const Texture& texture) {
    ambientTextures.push_back(texture);
}

void ros::Material::addEmissiveTexture(const Texture& texture) {
    emissiveTextures.push_back(texture);
}

void ros::Material::addGlossinessMap(const Texture& map) {
    glossinessMaps.push_back(map);
}

void ros::Material::addHeightMap(const Texture& map) {
    heightMaps.push_back(map);
}

void ros::Material::addNormalMap(const Texture& map) {
    normalMaps.push_back(map);
}

void ros::Material::addLightMap(const Texture& map) {
    lightMaps.push_back(map);
}
