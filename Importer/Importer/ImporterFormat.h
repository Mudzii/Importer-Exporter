#ifndef IMPORTER_FORMAT_H
#define IMPORTER_FORMAT_H

#include <fstream>
#include <iostream>
#include <vector>


#include "AnimatedMesh.h"
#include "StaticMesh.h"


// HEADER ==========================================================
class GRP4Header {
public: 

	struct VertexBlendingInfo {
		float blendingWeight;
		unsigned int blendingIndex;
	};

	struct ModelDataHeader {
		unsigned int staticMeshCount;
		unsigned int animatedMeshCount;
	};

	struct StaticMeshDataHeader {
		unsigned int vertexCount;
		unsigned int indexCount;
	};

	struct AnimatedMeshDataHeader {
		unsigned int animatedVertexCount;
		unsigned int animatedIndexCount;

		unsigned int jointCount;
		unsigned int animationCount;
		unsigned int keyframeCount;
	};

	struct StaticMeshControlPoint {
		float position[3];
	};

	struct AnimatedMeshControlPoint {
		float position[3];
		std::vector<VertexBlendingInfo> blendingInfo;
	};

	struct Triangle {
		std::vector<unsigned int> indices;
	};

public:
	friend class CustomFileLoader;


	char buff[255]; 
	int meshNameLength;
	std::vector<StaticMesh> staticMesh;
	std::vector<AnimatedMesh> animatedMesh;

public:
	GRP4Header(); 
	~GRP4Header(); 

	void CleanUp();

	// STATIC MESH ============= 
	int GetNrOfStaticMeshes(); 
	int GetStaticMeshIndex(int index); 

	int GetNrOfIndicesForStaticMesh(int meshIndex);
	int GetNrOfVerticiesForStaticMesh(int meshIndex); 

	std::string GetMeshNameForStaticMesh(int meshIndex);
	Mesh::Material GetMaterialForStaticMesh(int meshIndex);
	
	std::vector<unsigned int> GetIndicesForStaticMesh(int meshIndex); 
	std::vector<Mesh::StaticVertex> GetVerticiesForStaticMesh(int meshIndex);


	void CreateStaticModel(std::string meshName, int staticMeshIndex, const std::vector<Mesh::StaticVertex> &vertices,
						 const std::vector<unsigned int> &indices, Mesh::Material material);


	// ANIMATED MESH =========== 
	int GetNrOfAnimatedMeshes(); 
	int GetAnimatedMeshIndex(int index); 
	int GetNrOfVerticesForAnimatedMesh(int meshIndex); 

	std::string GetMeshNameForAnimatedMesh(int meshIndex);
	Mesh::Material GetMaterialForAnimatedMesh(int meshIndex); 

	std::vector<unsigned int> GetIndicesForAnimatedMesh(int meshIndex); 
	std::vector<Mesh::Joint> GetSkeletonForAnimatedMesh(int meshIndex); 
	std::vector<Mesh::Animation> GetAnimationsForAnimatedMesh(int meshIndex); 
	std::vector<Mesh::AnimatedVertex> GetVerticesForAnimatedMesh(int meshIndex); 
	std::vector<Mesh::Matrix4x4> GetKeyframesPackedForAnimatedMesh(int meshIndex); 
	std::vector<std::vector<std::vector<Mesh::Matrix4x4>>> GetKeyframesForAnimatedMesh(int meshIndex); 
	
	int FindJointIndexByName(const char* pName, std::vector<Mesh::Joint> & pSkeleton);

	void CreateAnimatedModel(std::string animatedMeshName, int animatedMeshIndex, const std::vector<Mesh::AnimatedVertex> &vertices,
		const std::vector<unsigned int> &indices, Mesh::Material material, const std::vector<Mesh::Joint> &skeleton, const std::vector<Mesh::Animation> &animations,
		const std::vector<std::vector<std::vector<Mesh::Matrix4x4>>> &keyframes);
};


// FILE LOADER =====================================================
class CustomFileLoader {

public:

	void SaveToFile(const char* fileName, GRP4Header &header);
	int LoadFromFile(const char* fileName, GRP4Header &header);

};

#endif // !IMPORTER_FORMAT_H