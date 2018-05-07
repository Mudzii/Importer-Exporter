#include "ImporterFormat.h"

GRP4Header::GRP4Header() {

}

GRP4Header::~GRP4Header() {

}

void GRP4Header::CleanUp() {

	this->staticMesh.clear(); 
	this->animatedMesh.clear();
}

// STATIC MESH ============= 

int GRP4Header::GetNrOfStaticMeshes() {
	return this->staticMesh.size(); 
}

int GRP4Header::GetStaticMeshIndex(int index) {
	return this->staticMesh[index].GetMeshIndex(); 
}

int GRP4Header::GetNrOfIndicesForStaticMesh(int meshIndex) {
	return this->staticMesh[meshIndex].GetIndexCount(); 
}

int GRP4Header::GetNrOfVerticiesForStaticMesh(int meshIndex) {
	return this->staticMesh[meshIndex].verticies.size(); 
}

std::string GRP4Header::GetMeshNameForStaticMesh(int meshIndex) {
	return this->staticMesh[meshIndex].GetMeshName().c_str(); 
}

Mesh::Material GRP4Header::GetMaterialForStaticMesh(int meshIndex) {
	return this->staticMesh[meshIndex].GetMaterial();
}

std::vector<unsigned int> GRP4Header::GetIndicesForStaticMesh(int meshIndex) {
	return this->staticMesh[meshIndex].GetIndices();
}

std::vector<Mesh::StaticVertex> GRP4Header::GetVerticiesForStaticMesh(int meshIndex) {
	return this->staticMesh[meshIndex].GetVerticies();
}

void GRP4Header::CreateStaticModel(std::string meshName, int staticMeshIndex, const std::vector<Mesh::StaticVertex> &vertices,
								 const std::vector<unsigned int> &indices, Mesh::Material material) {


	StaticMesh mesh = { meshName.c_str() , staticMeshIndex, vertices, indices, material };
	staticMesh.push_back(mesh);
}


// ANIMATED MESH =========== 

int GRP4Header::GetNrOfAnimatedMeshes() {
	return this->animatedMesh.size();
}

int GRP4Header::GetAnimatedMeshIndex(int index) {
	return this->animatedMesh[index].GetAnimatedMeshIndex(); 
}

int GRP4Header::GetNrOfVerticesForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].verticies.size(); 
}

Mesh::Material GRP4Header::GetMaterialForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetAnimatedMeshMaterial(); 
}

std::string GRP4Header::GetMeshNameForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetAnimatedMeshName().c_str();
}

std::vector<unsigned int> GRP4Header::GetIndicesForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetAnimatedIndices();
}

std::vector<Mesh::Joint> GRP4Header::GetSkeletonForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetAnimatedMeshJoints(); 
}

std::vector<Mesh::Animation> GRP4Header::GetAnimationsForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetAnimations(); 
}

std::vector<Mesh::AnimatedVertex> GRP4Header::GetVerticesForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetAnimatedVerticies();
}

std::vector<std::vector<std::vector<Mesh::Matrix4x4>>> GRP4Header::GetKeyframesForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetKeyframes(); 
}

std::vector<Mesh::Matrix4x4> GRP4Header::GetKeyframesPackedForAnimatedMesh(int meshIndex) {
	return this->animatedMesh[meshIndex].GetKeyframesPacked(); 
}

int GRP4Header::FindJointIndexByName(const char* pName, std::vector<Mesh::Joint>& pSkeleton) {
	int id = -1;

	for (unsigned int i = 0; i < pSkeleton.size(); i++)
	{
		if (strcmp(pSkeleton[i].boneName, pName) == 0)
		{
			id = i;
			break;
		}
	}

	return id;
}

void GRP4Header::CreateAnimatedModel(std::string animatedMeshName, int animatedMeshIndex, const std::vector<Mesh::AnimatedVertex> &vertices,
	const std::vector<unsigned int> &indices, Mesh::Material material, const std::vector<Mesh::Joint> &skeleton, const std::vector<Mesh::Animation> &animations,
	const std::vector<std::vector<std::vector<Mesh::Matrix4x4>>> &keyframes) {

	AnimatedMesh animMesh = {animatedMeshName.c_str(), animatedMeshIndex, vertices, indices, 
							 material, skeleton, animations, keyframes};

	animatedMesh.push_back(animMesh); 
}


// FILE LOADER =====================================================

void CustomFileLoader::SaveToFile(const char* fileName, GRP4Header &header) {

	// get mesh Counts
	GRP4Header::ModelDataHeader modelDataHeader; 
	modelDataHeader.animatedMeshCount = header.animatedMesh.size();
	modelDataHeader.staticMeshCount	  = header.staticMesh.size();

	//open file
	std::ofstream outFile(fileName, std::ofstream::binary);
	
	if (!outFile)
		std::cout << "Cold not write file!" << std::endl; 

	// write header
	outFile.write((const char*)&modelDataHeader, sizeof(GRP4Header::ModelDataHeader));

	// write static meshes =========================
	for (int statMeshIndex = 0; statMeshIndex < modelDataHeader.staticMeshCount; statMeshIndex++) {

		GRP4Header::StaticMeshDataHeader staticHeader; 
		staticHeader.vertexCount = header.staticMesh[statMeshIndex].verticies.size();
		staticHeader.indexCount  = header.staticMesh[statMeshIndex].indices.size();

		header.meshNameLength = header.staticMesh[statMeshIndex].GetMeshName().length();

		outFile.write((const char*)&staticHeader, sizeof(GRP4Header::StaticMeshDataHeader)); 

	/*	outFile.write((const char*)&header.staticMesh[statMeshIndex].staticMeshIndex, sizeof(int));
		outFile.write((const char*)&header.meshNameLength,							  sizeof(int));
		outFile.write((const char*)header.staticMesh[statMeshIndex].meshName.c_str(), header.meshNameLength);*/
		outFile.write((const char*)header.staticMesh[statMeshIndex].verticies.data(), header.staticMesh[statMeshIndex].verticies.size() * sizeof(Mesh::StaticVertex) );
		outFile.write((const char*)header.staticMesh[statMeshIndex].indices.data(),   header.staticMesh[statMeshIndex].indices.size()	* sizeof(unsigned int));
		outFile.write((const char*)&header.staticMesh[statMeshIndex].material,		  sizeof(Mesh::Material));

	}


	// write animated meshes =======================  
	for (int animMeshIndex = 0; animMeshIndex < modelDataHeader.animatedMeshCount; animMeshIndex++) {

		GRP4Header::AnimatedMeshDataHeader animatedHeader; 
		animatedHeader.jointCount		   = header.animatedMesh[animMeshIndex].skeleton.size(); 
		animatedHeader.animatedIndexCount  = header.animatedMesh[animMeshIndex].indicies.size();
		animatedHeader.animatedVertexCount = header.animatedMesh[animMeshIndex].verticies.size(); 
		animatedHeader.animationCount	   = header.animatedMesh[animMeshIndex].animations.size(); 
		
		header.meshNameLength = header.animatedMesh[animMeshIndex].animatedMeshName.length();

		header.animatedMesh[animMeshIndex].keyframesPackedData.clear(); 
		for (int animationInd = 0; animationInd < animatedHeader.animationCount; animationInd++) {
			for (int jointIndex = 0; jointIndex < animatedHeader.jointCount; jointIndex++) {
				
				header.animatedMesh[animMeshIndex].keyframesPackedData.insert(
					end(header.animatedMesh[animMeshIndex].keyframesPackedData),
					begin(header.animatedMesh[animMeshIndex].keyframes[animationInd][jointIndex]),
					end(header.animatedMesh[animMeshIndex].keyframes[animationInd][jointIndex]));

			}
		}


		animatedHeader.keyframeCount = header.animatedMesh[animMeshIndex].keyframesPackedData.size(); 

		outFile.write((const char*)&animatedHeader, sizeof(GRP4Header::AnimatedMeshDataHeader));

		/*outFile.write((const char*)&header.animatedMesh[animMeshIndex].animatedMeshIndex,		  sizeof(int));
		outFile.write((const char*)&header.meshNameLength,										  sizeof(int));
		outFile.write((const char*)header.animatedMesh[animMeshIndex].animatedMeshName.c_str(),   header.meshNameLength);*/
		outFile.write((const char*)header.animatedMesh[animMeshIndex].verticies.data(),			  animatedHeader.animatedVertexCount * sizeof(Mesh::AnimatedVertex) );
		outFile.write((const char*)header.animatedMesh[animMeshIndex].indicies.data(),			  animatedHeader.animatedIndexCount  * sizeof(unsigned int));
		outFile.write((const char*)header.animatedMesh[animMeshIndex].skeleton.data(),			  animatedHeader.jointCount			 * sizeof(Mesh::Joint));
		outFile.write((const char*)header.animatedMesh[animMeshIndex].animations.data(),		  animatedHeader.animationCount		 * sizeof(Mesh::Animation));
		outFile.write((const char*)header.animatedMesh[animMeshIndex].keyframesPackedData.data(), animatedHeader.keyframeCount		 * sizeof(Mesh::Matrix4x4));
		outFile.write((const char*)&header.animatedMesh[animMeshIndex].materials,				  sizeof(Mesh::Material)); 
	
	}


	//close file
	outFile.close();

}

int CustomFileLoader::LoadFromFile(const char* fileName, GRP4Header &header) {

	int nrOfMeshes = 0;

	//open filestream
	std::ifstream inFile(fileName, std::ifstream::binary);

	if (!inFile)
		std::cout << "could not read from file!" << std::endl;

	//read header
	GRP4Header::ModelDataHeader modelDataHeader;
	inFile.read((char*)&modelDataHeader, sizeof(GRP4Header::ModelDataHeader));

	header.staticMesh.clear();
	header.staticMesh.resize(modelDataHeader.staticMeshCount);

	header.animatedMesh.clear(); 
	header.animatedMesh.resize(modelDataHeader.animatedMeshCount);


	// read static meshes ==========================  
	for (int statMeshIndex = 0; statMeshIndex < modelDataHeader.staticMeshCount; statMeshIndex++) {

		header.staticMesh[statMeshIndex].staticMeshIndex = statMeshIndex; 

		memset(header.buff, 0, 255);
		GRP4Header::StaticMeshDataHeader staticHeader;

		inFile.read((char*)&staticHeader, sizeof(GRP4Header::StaticMeshDataHeader));

		header.staticMesh[statMeshIndex].verticies.resize(staticHeader.vertexCount);
		header.staticMesh[statMeshIndex].indices.resize(staticHeader.indexCount);

		//inFile.read((char*)&header.staticMesh[statMeshIndex].staticMeshIndex, sizeof(int));
		//inFile.read((char*)&header.meshNameLength,							  sizeof(int));
		//inFile.read(header.buff,											  header.meshNameLength);
		inFile.read((char*)header.staticMesh[statMeshIndex].verticies.data(), staticHeader.vertexCount * sizeof(Mesh::StaticVertex));
		inFile.read((char*)header.staticMesh[statMeshIndex].indices.data(),   staticHeader.indexCount  * sizeof(unsigned int));
		inFile.read((char*)&header.staticMesh[statMeshIndex].material,		  sizeof(Mesh::Material));

		header.staticMesh[statMeshIndex].meshName.assign(header.buff);
		nrOfMeshes++; 

	}

	// read animated meshes ========================  
	for (int animMeshIndex = 0; animMeshIndex < modelDataHeader.animatedMeshCount; animMeshIndex++) {

		memset(header.buff, 0, 255); 
		unsigned int keyframeOffset = 0; 
		unsigned int animationLength = 0; 
		header.animatedMesh[animMeshIndex].animatedMeshIndex = animMeshIndex;

		GRP4Header::AnimatedMeshDataHeader animatedHeader;
		inFile.read((char*)&animatedHeader, sizeof(GRP4Header::AnimatedMeshDataHeader));

		header.animatedMesh[animMeshIndex].skeleton.resize(animatedHeader.jointCount);
		header.animatedMesh[animMeshIndex].animations.resize(animatedHeader.animationCount); 
		header.animatedMesh[animMeshIndex].indicies.resize(animatedHeader.animatedIndexCount);
		header.animatedMesh[animMeshIndex].keyframes.resize(animatedHeader.animationCount);
		header.animatedMesh[animMeshIndex].verticies.resize(animatedHeader.animatedVertexCount);
		header.animatedMesh[animMeshIndex].keyframesPackedData.resize(animatedHeader.keyframeCount);

		//inFile.read((char*)&header.animatedMesh[animMeshIndex].animatedMeshIndex,		  sizeof(int));
		//inFile.read((char*)&header.meshNameLength,										  sizeof(int));
		//inFile.read(header.buff,														  header.meshNameLength);
		//header.animatedMesh[animMeshIndex].animatedMeshName.assign(header.buff);
		inFile.read((char*)header.animatedMesh[animMeshIndex].verticies.data(),			  animatedHeader.animatedVertexCount * sizeof(Mesh::AnimatedVertex));
		inFile.read((char*)header.animatedMesh[animMeshIndex].indicies.data(),			  animatedHeader.animatedIndexCount  * sizeof(unsigned int)); 
		inFile.read((char*)header.animatedMesh[animMeshIndex].skeleton.data(),			  animatedHeader.jointCount * sizeof(Mesh::Joint)); 
		inFile.read((char*)header.animatedMesh[animMeshIndex].animations.data(),		  animatedHeader.animationCount * sizeof(Mesh::Animation));
		inFile.read((char*)header.animatedMesh[animMeshIndex].keyframesPackedData.data(), animatedHeader.keyframeCount  * sizeof(Mesh::Matrix4x4));
		inFile.read((char*)&header.animatedMesh[animMeshIndex].materials,				  sizeof(Mesh::Material)); 

		for (int animationIndex = 0; animationIndex < animatedHeader.animationCount; animationIndex++) {
			
			header.animatedMesh[animMeshIndex].keyframes[animationIndex].resize(animatedHeader.jointCount);
			animationLength = header.animatedMesh[animMeshIndex].animations[animationIndex].nrOfFrames;

			for (int jointIndex = 0; jointIndex < animatedHeader.jointCount; jointIndex++) {
		 	
				header.animatedMesh[animMeshIndex].keyframes[animationIndex][jointIndex].clear();

				header.animatedMesh[animMeshIndex].keyframes[animationIndex][jointIndex].insert(
					begin(header.animatedMesh[animMeshIndex].keyframes[animationIndex][jointIndex]),
					begin(header.animatedMesh[animMeshIndex].keyframesPackedData) + keyframeOffset,
					begin(header.animatedMesh[animMeshIndex].keyframesPackedData) + keyframeOffset + animationLength);

				keyframeOffset += animationLength;

			}
		}
		//nrOfMeshes++; 
	}


	inFile.close();
	return nrOfMeshes;
}




