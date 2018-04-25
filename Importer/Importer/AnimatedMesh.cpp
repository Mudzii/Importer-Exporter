#include "AnimatedMesh.h"

AnimatedMesh::AnimatedMesh(std::string animatedMeshName, int animatedMeshIndex, const std::vector<Mesh::AnimatedVertex>& vertices,
	const std::vector<unsigned int>& indices, Mesh::Material materials, const std::vector<Mesh::Joint> skeleton, const std::vector<Mesh::Animation> animations,
	const std::vector<std::vector<std::vector<Mesh::Matrix4x4>>> &keyframes)
{
	this->animatedMeshName = animatedMeshName; 
	this->animatedMeshIndex = animatedMeshIndex; 

	this->verticies  = vertices; 
	this->indicies   = indices; 
	this->materials  = materials;

	this->skeleton   = skeleton; 
	this->animations = animations; 
	this->keyframes  = keyframes; 

}

AnimatedMesh::AnimatedMesh() {

	this->animatedMeshName = ""; 
	this->animatedMeshIndex = 0; 
}

AnimatedMesh::~AnimatedMesh() {

}

// =============
int AnimatedMesh::GetAnimatedMeshIndex() {
	return this->animatedMeshIndex; 
}

int AnimatedMesh::GetKeyframeCount() {
	return this->keyframes.size(); 
}

int AnimatedMesh::GetAnimationCount() {
	return this->animations.size(); 
}

int AnimatedMesh::GetAnimatedIndexCount() {
	return this->indicies.size();
}

int AnimatedMesh::GetAnimatedJointCount() {
	return this->skeleton.size(); 
}

int AnimatedMesh::GetAnimatedVertexCount() {
	return this->verticies.size();
}

std::string AnimatedMesh::GetAnimatedMeshName() {
	return this->animatedMeshName; 
}

Mesh::Material AnimatedMesh::GetAnimatedMeshMaterial() {
	return this->materials; 
}

std::vector<Mesh::Animation> AnimatedMesh::GetAnimations()
{
	return this->animations; 
}

std::vector<Mesh::Joint> AnimatedMesh::GetAnimatedMeshJoints() {
	return this->skeleton; 
}

std::vector<unsigned int> AnimatedMesh::GetAnimatedIndices() {
	return this->indicies; 
}

std::vector<Mesh::AnimatedVertex> AnimatedMesh::GetAnimatedVerticies() {
	return this->verticies; 
}

std::vector<std::vector<std::vector<Mesh::Matrix4x4>>> AnimatedMesh::GetKeyframes()
{
	return this->keyframes; 
}

std::vector<Mesh::Matrix4x4> AnimatedMesh::GetKeyframesPacked() {
	return this->keyframesPackedData; 
}

// =============

void AnimatedMesh::PushBackIndice(unsigned int indice) {
	this->indicies.push_back(indice);
}

void AnimatedMesh::PushBackVertex(AnimatedVertex vertex) {
	this->verticies.push_back(vertex); 
}



// =============

void AnimatedMesh::SetAnimatedMeshName(std::string animatedMeshName) {
	this->animatedMeshName = animatedMeshName; 
}

void AnimatedMesh::SetAnimatedMeshMaterial(Mesh::Material animatedMaterial) {
	this->materials = materials; 
}

void AnimatedMesh::SetAnimatedMeshIndex(unsigned int animatedMeshIndex) {
	this->animatedMeshIndex = animatedMeshIndex; 
}