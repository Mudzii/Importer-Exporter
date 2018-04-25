#ifndef ANIMATEDMESH_H
#define ANIMATEDMESH_H

#include <stdio.h>
#include <vector>
#include "Mesh.h"


class AnimatedMesh : public Mesh {
public:
	std::string animatedMeshName;
	int animatedMeshIndex;

	std::vector<AnimatedVertex> verticies;
	std::vector<unsigned int> indicies;
	Material materials;

	std::vector<Joint> skeleton;
	std::vector<Animation> animations;
	std::vector<std::vector<std::vector<Matrix4x4>>> keyframes;
	std::vector<Matrix4x4> keyframesPackedData;
	Material animatedMaterial;

public:
	AnimatedMesh(std::string animatedMeshName, int animatedMeshIndex, const std::vector<Mesh::AnimatedVertex>& vertices,
		const std::vector<unsigned int>& indices, Mesh::Material materials, const std::vector<Mesh::Joint> skeleton, const std::vector<Mesh::Animation> animations,
		const std::vector<std::vector<std::vector<Mesh::Matrix4x4>>> &keyframes);
	~AnimatedMesh();
	AnimatedMesh(); 

	// =============

	int GetAnimatedMeshIndex();

	int GetKeyframeCount(); 
	int GetAnimationCount(); 
	int GetAnimatedJointCount(); 
	int GetAnimatedIndexCount();
	int GetAnimatedVertexCount();


	std::string GetAnimatedMeshName();
	Material GetAnimatedMeshMaterial(); 
	std::vector<Animation> GetAnimations(); 
	std::vector<Joint> GetAnimatedMeshJoints(); 
	std::vector<unsigned int> GetAnimatedIndices();
	std::vector<AnimatedVertex> GetAnimatedVerticies();
	std::vector<std::vector<std::vector<Matrix4x4>>> GetKeyframes();
	std::vector<Mesh::Matrix4x4> GetKeyframesPacked(); 

	// =============

	void PushBackIndice(unsigned int indice);
	void PushBackVertex(AnimatedVertex vertex);

	// =============

	void SetAnimatedMeshName(std::string animatedMeshName);
	void SetAnimatedMeshMaterial(Material animatedMaterial);
	void SetAnimatedMeshIndex(unsigned int animatedMeshIndex);

};

#endif // ! ANIMATEDMESH_H

