#define _CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <fbxsdk.h>
#include <algorithm>

#include <Importer\ImporterFormat.h>
#pragma comment(lib, "Importer.lib")
#pragma comment(lib, "libfbxsdk-md.lib")

using namespace std;

// =====================================================================

FbxAMatrix getGeometryTransformation(FbxNode* lNode);
Mesh::Material getMaterial(FbxNode* lNode, GRP4Header& header);

void staticMesh(FbxNode* lNode, GRP4Header& header, const char* name);
void animatedMesh(FbxNode* lNode, GRP4Header& header, FbxScene* lScene, const char* name);

void FbxMat4ToMat4x4(const FbxAMatrix& mat4, Mesh::Matrix4x4& newMat);
void ProcessSkeleton(FbxNode * pNode, std::vector<Mesh::Joint> & pSkeleton);
void ProcessSkeletonRecursively(FbxNode * pNode, int pParentIndex, std::vector<Mesh::Joint> & pSkeleton);
void processJointsAndAnimations(FbxNode * pNode, std::vector<GRP4Header::AnimatedMeshControlPoint> & pControlPoints, AnimatedMesh & pAnimatedMeshData, std::vector<Mesh::Joint> & pSkeleton, GRP4Header& header, FbxScene* lScene);

bool CheckIfNodeHasSkeleton(FbxNode* pNode);
bool CheckSkeletonRecursively(FbxNode* pNode);
bool BlendingInfoSort(GRP4Header::VertexBlendingInfo const& lhs, GRP4Header::VertexBlendingInfo const& rhs);

// =====================================================================

void FbxMat4ToMat4x4(const FbxAMatrix& mat4, Mesh::Matrix4x4& newMat)
{
	newMat.column1[0] = mat4[0][0];
	newMat.column1[1] = mat4[0][1];
	newMat.column1[2] = mat4[0][2];
	newMat.column1[3] = mat4[0][3];

	newMat.column2[0] = mat4[1][0];
	newMat.column2[1] = mat4[1][1];
	newMat.column2[2] = mat4[1][2];
	newMat.column2[3] = mat4[1][3];

	newMat.column3[0] = mat4[2][0];
	newMat.column3[1] = mat4[2][1];
	newMat.column3[2] = mat4[2][2];
	newMat.column3[3] = mat4[2][3];

	newMat.column4[0] = mat4[3][0];
	newMat.column4[1] = mat4[3][1];
	newMat.column4[2] = mat4[3][2];
	newMat.column4[3] = mat4[3][3];
}

void processJointsAndAnimations(FbxNode * pNode, std::vector<GRP4Header::AnimatedMeshControlPoint> & pControlPoints, AnimatedMesh & pAnimatedMeshData, std::vector<Mesh::Joint> & pSkeleton, GRP4Header& header, FbxScene* lScene)
{
	FbxMesh* currentMesh = pNode->GetMesh();
	unsigned int numOfDeformers = currentMesh->GetDeformerCount();

	FbxAMatrix geometryTransform = getGeometryTransformation(pNode);

	// Assume only one skeleton per mesh
	FbxSkin* currentSkin = reinterpret_cast<FbxSkin*>(currentMesh->GetDeformer(0, FbxDeformer::eSkin));
	if (!currentSkin)
		return;

	int numJoints = currentSkin->GetClusterCount();
	std::cout << "Nr of joints: " << numJoints << std::endl;

	for (unsigned int jointIndex = 0; jointIndex < numJoints; jointIndex++)
	{
		FbxCluster * currentJoint = currentSkin->GetCluster(jointIndex);
		const char* currentJointName = currentJoint->GetLink()->GetName();

		//std::cout << "Jnt name: " << currentJointName << std::endl; 

		int currentJointIndex = header.FindJointIndexByName(currentJointName, pSkeleton);

		if (currentJointIndex == -1)
			return;

		// Associate each joint with the control points it affects
		unsigned int numOfIndices = currentJoint->GetControlPointIndicesCount();
		for (unsigned int i = 0; i < numOfIndices; i++)
		{
			GRP4Header::VertexBlendingInfo currentBlendingInfo;
			currentBlendingInfo.blendingIndex = currentJointIndex;
			currentBlendingInfo.blendingWeight = currentJoint->GetControlPointWeights()[i];
			pControlPoints[currentJoint->GetControlPointIndices()[i]].blendingInfo.push_back(currentBlendingInfo);
		}
	}

	// Get animation information
	int numOfAnimations = lScene->GetSrcObjectCount<FbxAnimStack>();
	for (int animationIndex = 0; animationIndex < numOfAnimations; animationIndex++)
	{
		pAnimatedMeshData.animations.push_back(Mesh::Animation());
		pAnimatedMeshData.keyframes.push_back(std::vector<std::vector<Mesh::Matrix4x4>>());

		FbxAnimStack * currentAnimStack = lScene->GetSrcObject<FbxAnimStack>(animationIndex);
		FbxString animationStackName = currentAnimStack->GetName();
		std::string animationName = animationStackName.Buffer();
		FbxTakeInfo * takeInfo = lScene->GetTakeInfo(animationStackName);
		FbxTime start = takeInfo->mLocalTimeSpan.GetStart();
		FbxTime end = takeInfo->mLocalTimeSpan.GetStop();
		int animationLength = end.GetFrameCount(FbxTime::eFrames24) - start.GetFrameCount(FbxTime::eFrames24) + 1;

		pAnimatedMeshData.animations.back().nrOfFrames = animationLength;
		pAnimatedMeshData.animations.back().animationIndex = animationIndex;
		_snprintf_s(pAnimatedMeshData.animations.back().animationName, sizeof(pAnimatedMeshData.animations.back().animationName), animationName.c_str());

		unsigned int numJoints = currentSkin->GetClusterCount();
		for (unsigned int jointIndex = 0; jointIndex < numJoints; jointIndex++)
		{
			FbxCluster * currentJoint = currentSkin->GetCluster(jointIndex);
			const char* currentJointName = currentJoint->GetLink()->GetName();
			int currentJointIndex = header.FindJointIndexByName(currentJointName, pSkeleton);

			if (currentJointIndex == -1)
				return;

			//std::cout << std::to_string(currentJointIndex) << std::endl;

			FbxAMatrix transformMatrix;
			FbxAMatrix transformLinkMatrix;
			FbxAMatrix globalBindposeInverseMatrix;

			currentJoint->GetTransformMatrix(transformMatrix);	// The transformation of the mesh at binding time
			currentJoint->GetTransformLinkMatrix(transformLinkMatrix);	// The transformation of the cluster(joint) at binding time from joint space to world space
			globalBindposeInverseMatrix = transformLinkMatrix.Inverse() * transformMatrix * geometryTransform;

			std::vector<Mesh::Matrix4x4> keyframes;

			pAnimatedMeshData.keyframes.back().push_back(std::vector<Mesh::Matrix4x4>(animationLength));


			for (FbxLongLong frameIndex = start.GetFrameCount(FbxTime::eFrames24); frameIndex <= end.GetFrameCount(FbxTime::eFrames24); frameIndex++)
			{
				FbxTime currentTime;
				currentTime.SetFrame(frameIndex, FbxTime::eFrames24);
				FbxAMatrix currentTransformOffset = pNode->EvaluateGlobalTransform(currentTime) * geometryTransform;
				FbxAMatrix globalTransform = currentTransformOffset.Inverse() * currentJoint->GetLink()->EvaluateGlobalTransform(currentTime);
				FbxAMatrix finalFBXTransform = globalTransform * globalBindposeInverseMatrix;

				Mesh::Matrix4x4 finalTransform;
				FbxMat4ToMat4x4(finalFBXTransform, finalTransform);
				keyframes.push_back(finalTransform);
			}

			pAnimatedMeshData.keyframes[animationIndex][jointIndex] = keyframes;
		}
	}

	// Some of the control points only have less than 4 joints
	// affecting them.
	// For a normal renderer, there are usually 4 joints
	// I am adding more dummy joints if there isn't enough
	GRP4Header::VertexBlendingInfo currentBlendingInfo;
	currentBlendingInfo.blendingIndex = 0;
	currentBlendingInfo.blendingWeight = 0.0f;
	for (auto & ctrlPoint : pControlPoints)
	{
		for (size_t i = ctrlPoint.blendingInfo.size(); i < 4; i++)
		{
			ctrlPoint.blendingInfo.push_back(currentBlendingInfo);
		}
	}
}

Mesh::Material getMaterial(FbxNode* lNode, GRP4Header& header)
{
	Mesh::Material meshMaterial = Mesh::Material();

	for (int i = 0; i < lNode->GetSrcObjectCount<FbxSurfaceMaterial>(); i++)
	{
		FbxSurfaceMaterial* material = (FbxSurfaceMaterial*)lNode->GetSrcObject<FbxSurfaceMaterial>(i);
		std::string texturRelativePath;
		std::string textureName;

		if (material)
		{
			FbxProperty diffuse = material->FindProperty(FbxSurfaceMaterial::sDiffuse);

			// Directly get textures
			int textureCount = diffuse.GetSrcObjectCount<FbxTexture>();
			for (int j = 0; j < textureCount; j++)
			{
				const FbxFileTexture* texture = FbxCast<FbxFileTexture>(diffuse.GetSrcObject<FbxTexture>(j));
				// Then, you can get all the properties of the texture, include its name
				texturRelativePath = texture->GetRelativeFileName();
				size_t endPathPos = texturRelativePath.find_last_of("\\");
				if (endPathPos != std::string::npos)
				{
					textureName = texturRelativePath.substr(endPathPos + 1);
					_snprintf_s(meshMaterial.diffuseTexture, sizeof(meshMaterial.diffuseTexture), textureName.c_str());
					std::cout << "Diffuse texture: " << meshMaterial.diffuseTexture << std::endl;
				}
			}

			FbxProperty bump = material->FindProperty(FbxSurfaceMaterial::sBump);

			// Directly get textures
			textureCount = bump.GetSrcObjectCount<FbxTexture>();
			for (int j = 0; j < textureCount; j++)
			{
				const FbxFileTexture* texture = FbxCast<FbxFileTexture>(bump.GetSrcObject<FbxTexture>(j));
				// Then, you can get all the properties of the texture, include its name
				texturRelativePath = texture->GetRelativeFileName();
				size_t endPathPos = texturRelativePath.find_last_of("\\");
				if (endPathPos != std::string::npos)
				{
					textureName = texturRelativePath.substr(endPathPos + 1);
					_snprintf_s(meshMaterial.normalTexture, sizeof(meshMaterial.normalTexture), textureName.c_str());
					std::cout << "Normal texture: " << meshMaterial.normalTexture << std::endl;
				}
			}
		}
	}

	return meshMaterial;
}

FbxAMatrix getGeometryTransformation(FbxNode* lNode)
{
	if (!lNode)
	{
		std::cout << "Null for mesh geometry" << std::endl;
	}

	const FbxVector4 lT = lNode->GetGeometricTranslation(FbxNode::eSourcePivot);
	const FbxVector4 lR = lNode->GetGeometricRotation(FbxNode::eSourcePivot);
	const FbxVector4 lS = lNode->GetGeometricScaling(FbxNode::eSourcePivot);

	return FbxAMatrix(lT, lR, lS);
}

void staticMesh(FbxNode* lNode, GRP4Header& header, const char* name)
{
	StaticMesh tempMesh;

	// Use node to get information about mesh
	FbxMesh* lMesh = lNode->GetMesh();
	FbxVector4* lVertexArray = lMesh->GetControlPoints();
	FbxGeometryElementNormal* lNormal = lMesh->GetElementNormal();
	FbxGeometryElementTangent* lTangent = lMesh->GetElementTangent();
	FbxGeometryElementBinormal* lBiNormal = lMesh->GetElementBinormal();
	FbxGeometryElementUV* lVertUV = lMesh->GetElementUV();


	unsigned int ctrlPointCount = lMesh->GetControlPointsCount();

	if (ctrlPointCount == 0)
		return;


	vector<Mesh::StaticVertex> staticVerts;
	vector<GRP4Header::StaticMeshControlPoint> meshCtrlPoints;

	const char* meshName = lNode->GetName();
	std::cout << "Mesh name: " << meshName << std::endl;
	std::cout << "Vertices: " << ctrlPointCount << std::endl;
	std::cout << "Triangles: " << lMesh->GetPolygonCount() << std::endl;

	for (int i = 0; i < ctrlPointCount; i++)
	{
		GRP4Header::StaticMeshControlPoint ctrlPoint;

		ctrlPoint.position[0] = static_cast<float>(lVertexArray[i][0]);
		ctrlPoint.position[1] = static_cast<float>(lVertexArray[i][1]);
		ctrlPoint.position[2] = static_cast<float>(lVertexArray[i][2]);

		meshCtrlPoints.push_back(ctrlPoint);
	}

	int vertCount = 0;
	// Save mesh triangles in vectors
	for (int triangleIndex = 0; triangleIndex < lMesh->GetPolygonCount(); triangleIndex++)
	{
		for (int tVertIndex = 0; tVertIndex < 3; tVertIndex++)
		{
			int polyVertIndex = lMesh->GetPolygonVertex(triangleIndex, tVertIndex);

			Mesh::StaticVertex tempVert;
			tempVert.position[0] = meshCtrlPoints[polyVertIndex].position[0];
			tempVert.position[1] = meshCtrlPoints[polyVertIndex].position[1];
			tempVert.position[2] = meshCtrlPoints[polyVertIndex].position[2];

			FbxVector4 lNormals = lNormal->GetDirectArray().GetAt(vertCount);
			tempVert.normal[0] = static_cast<float>(lNormals[0]);
			tempVert.normal[1] = static_cast<float>(lNormals[1]);
			tempVert.normal[2] = static_cast<float>(lNormals[2]);

			FbxVector4 lTangents = lTangent->GetDirectArray().GetAt(vertCount);
			tempVert.tangent[0] = static_cast<float>(lTangents[0]);
			tempVert.tangent[1] = static_cast<float>(lTangents[1]);
			tempVert.tangent[2] = static_cast<float>(lTangents[2]);

			FbxVector4 lBiNormals = lBiNormal->GetDirectArray().GetAt(vertCount);
			tempVert.bitangent[0] = static_cast<float>(lBiNormals[0]);
			tempVert.bitangent[1] = static_cast<float>(lBiNormals[1]);
			tempVert.bitangent[2] = static_cast<float>(lBiNormals[2]);

			int index = lVertUV->GetIndexArray().GetAt(vertCount);

			FbxVector2 lVertUVs = lVertUV->GetDirectArray().GetAt(index);
			tempVert.UV[0] = static_cast<float>(lVertUVs[0]);
			tempVert.UV[1] = static_cast<float>(lVertUVs[1]);

			tempMesh.verticies.push_back(tempVert);
			tempMesh.indices.push_back(vertCount);
			vertCount++;
		}
	}

	tempMesh.SetMeshMaterial(getMaterial(lNode, header));
	header.CreateStaticModel(name, 0, tempMesh.verticies, tempMesh.indices, tempMesh.material);

}

void animatedMesh(FbxNode* lNode, GRP4Header& header, FbxScene* lScene, const char* name)
{
	AnimatedMesh tempMesh;
	//StaticMesh* tempMesh;
	GRP4Header::ModelDataHeader mDataHeader;

	mDataHeader.animatedMeshCount = 1;
	// Use node to get information about mesh
	FbxMesh* lMesh = lNode->GetMesh();
	FbxVector4* lVertexArray = lMesh->GetControlPoints();
	FbxGeometryElementNormal* lNormal = lMesh->GetElementNormal();
	FbxGeometryElementTangent* lTangent = lMesh->GetElementTangent();
	FbxGeometryElementBinormal* lBiNormal = lMesh->GetElementBinormal();
	FbxGeometryElementUV* lVertUV = lMesh->GetElementUV();

	//vector<Mesh::AnimatedVertex> verts;
	vector<GRP4Header::AnimatedMeshControlPoint> meshCtrlPoints;

	cout << "Vertices: " << lMesh->GetControlPointsCount() << endl;
	cout << "Triangles: " << lMesh->GetPolygonCount() << endl;
	for (int i = 0; i < lMesh->GetControlPointsCount(); i++)
	{
		GRP4Header::AnimatedMeshControlPoint ctrlPoint;

		ctrlPoint.position[0] = static_cast<float>(lVertexArray[i][0]);
		ctrlPoint.position[1] = static_cast<float>(lVertexArray[i][1]);
		ctrlPoint.position[2] = static_cast<float>(lVertexArray[i][2]);

		meshCtrlPoints.push_back(ctrlPoint);
	}


	ProcessSkeleton(lNode->GetParent(), tempMesh.skeleton);
	processJointsAndAnimations(lNode, meshCtrlPoints, tempMesh, tempMesh.skeleton, header, lScene);

	int vertCount = 0;
	// Save mesh triangles in vectors
	for (int triangleIndex = 0; triangleIndex < lMesh->GetPolygonCount(); triangleIndex++)
	{
		for (int tVertIndex = 0; tVertIndex < 3; tVertIndex++)
		{
			int ctrlPointIndex = lMesh->GetPolygonVertex(triangleIndex, tVertIndex);

			int polyVertIndex = lMesh->GetPolygonVertex(triangleIndex, tVertIndex);

			Mesh::AnimatedVertex tempVert;
			tempVert.position[0] = meshCtrlPoints[polyVertIndex].position[0];
			tempVert.position[1] = meshCtrlPoints[polyVertIndex].position[1];
			tempVert.position[2] = meshCtrlPoints[polyVertIndex].position[2];

			FbxVector4 lNormals = lNormal->GetDirectArray().GetAt(vertCount);
			tempVert.normal[0] = static_cast<float>(lNormals[0]);
			tempVert.normal[1] = static_cast<float>(lNormals[1]);
			tempVert.normal[2] = static_cast<float>(lNormals[2]);

			FbxVector4 lTangents = lTangent->GetDirectArray().GetAt(vertCount);
			tempVert.tangent[0] = static_cast<float>(lTangents[0]);
			tempVert.tangent[1] = static_cast<float>(lTangents[1]);
			tempVert.tangent[2] = static_cast<float>(lTangents[2]);

			FbxVector4 lBiNormals = lBiNormal->GetDirectArray().GetAt(vertCount);
			tempVert.bitangent[0] = static_cast<float>(lBiNormals[0]);
			tempVert.bitangent[1] = static_cast<float>(lBiNormals[1]);
			tempVert.bitangent[2] = static_cast<float>(lBiNormals[2]);

			int index = lVertUV->GetIndexArray().GetAt(vertCount);
			FbxVector2 lVertUVs = lVertUV->GetDirectArray().GetAt(index);
			tempVert.UV[0] = static_cast<float>(lVertUVs[0]);
			tempVert.UV[1] = static_cast<float>(lVertUVs[1]);

			std::vector<GRP4Header::VertexBlendingInfo> blendInfos;
			for (unsigned int i = 0; i < meshCtrlPoints[ctrlPointIndex].blendingInfo.size(); ++i)
			{
				GRP4Header::VertexBlendingInfo currBlendingInfo;
				currBlendingInfo.blendingIndex = meshCtrlPoints[ctrlPointIndex].blendingInfo[i].blendingIndex;
				currBlendingInfo.blendingWeight = meshCtrlPoints[ctrlPointIndex].blendingInfo[i].blendingWeight;
				blendInfos.push_back(currBlendingInfo);
			}

			std::sort(blendInfos.begin(), blendInfos.end(), &BlendingInfoSort);

			tempVert.boneIndicies[0] = blendInfos[0].blendingIndex;
			tempVert.boneIndicies[1] = blendInfos[1].blendingIndex;
			tempVert.boneIndicies[2] = blendInfos[2].blendingIndex;
			tempVert.boneIndicies[3] = blendInfos[3].blendingIndex;

			tempVert.boneWeight[0] = blendInfos[0].blendingWeight;
			tempVert.boneWeight[1] = blendInfos[1].blendingWeight;
			tempVert.boneWeight[2] = blendInfos[2].blendingWeight;
			tempVert.boneWeight[3] = blendInfos[3].blendingWeight;

			tempMesh.verticies.push_back(tempVert);
			tempMesh.indicies.push_back(vertCount);
			vertCount++;

		}

	}

	tempMesh.SetAnimatedMeshMaterial(getMaterial(lNode, header));

	header.CreateAnimatedModel(name, 0, tempMesh.verticies,
		tempMesh.indicies, tempMesh.animatedMaterial, tempMesh.skeleton, tempMesh.animations,
		tempMesh.keyframes);
}

void ProcessSkeleton(FbxNode * pNode, std::vector<Mesh::Joint> & pSkeleton)
{
	for (int childIndex = 0; childIndex < pNode->GetChildCount(); childIndex++)
	{
		FbxNode * currentNode = pNode->GetChild(childIndex);
		ProcessSkeletonRecursively(currentNode, -1, pSkeleton);
	}
}

void ProcessSkeletonRecursively(FbxNode * pNode, int pParentIndex, std::vector<Mesh::Joint> & pSkeleton)
{
	if (pNode->GetNodeAttribute() && pNode->GetNodeAttribute()->GetAttributeType() && pNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton)
	{
		Mesh::Joint currentJoint;
		memset(currentJoint.boneName, 0, 50);

		currentJoint.parentIndex = pParentIndex;
		currentJoint.jointIndex = pSkeleton.size();
		_snprintf_s(currentJoint.boneName, sizeof(currentJoint.boneName), pNode->GetName());
		pSkeleton.push_back(currentJoint);
		/*std::cout << "Joint: " << currentJoint.name << " | Parent: " << std::to_string(currentJoint.parentIndex)
		<< " | Myself: " << std::to_string(currentJoint.index) << std::endl;*/
	 }
	for (int i = 0; i < pNode->GetChildCount(); i++)
	{
		ProcessSkeletonRecursively(pNode->GetChild(i), pSkeleton.size() - 1, pSkeleton);
	}
}

bool BlendingInfoSort(GRP4Header::VertexBlendingInfo const& lhs, GRP4Header::VertexBlendingInfo const& rhs) {
	return lhs.blendingWeight > rhs.blendingWeight;
}

bool CheckSkeletonRecursively(FbxNode* pNode)
{
	bool result = false;

	if (pNode->GetNodeAttribute() && pNode->GetNodeAttribute()->GetAttributeType() && pNode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton)
		result = true;

	for (int i = 0; i < pNode->GetChildCount(); i++)
		result = CheckSkeletonRecursively(pNode->GetChild(i));

	return result;
}

bool CheckIfNodeHasSkeleton(FbxNode* pNode)
{
	bool result = false;

	for (int childIndex = 0; childIndex < pNode->GetChildCount(); childIndex++) {
		FbxNode * currentNode = pNode->GetChild(childIndex);
		result = CheckSkeletonRecursively(currentNode);
	}

	return result;
}

// =====================================================================


int main(int argc, char** argv)
{
	/*
	BEFORE USING:
	- remember to triangulate mesh
	- freeze transformations on meshes

	*/
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	fprintf(stderr, "%d\n", argc);
	fprintf(stderr, "%s\n", argv[0]);

	//fprintf(stderr, "%s\n", strstr(argv[1], "_anim"));

	//fprintf(stderr, "%s\n", argv[2]);
	getchar();
	return 0;


	CustomFileLoader importr;
	GRP4Header header;

	const char* fileName = "jumpStartLong.fbx";
	const char* filePath = "Materials/jumpStartLong.fbx";
	const char* newName = "jumpStartLong.vkp";

	bool isTriangulated = true;

	FbxManager* lSDKManager = FbxManager::Create();

	FbxIOSettings *ios = FbxIOSettings::Create(lSDKManager, IOSROOT);
	lSDKManager->SetIOSettings(ios);

	// Create an importer using the SDK manager.
	FbxImporter* lImporter = FbxImporter::Create(lSDKManager, "");

	// Use the first argument as the filename for the importer.
	if (!lImporter->Initialize(filePath, -1, lSDKManager->GetIOSettings())) {

		std::cout << "Call to FbxImporter::Initialize() failed" << std::endl;
		exit(-1);
	}

	// Create a new scene so that it can be populated by the imported file.
	FbxScene* lScene = FbxScene::Create(lSDKManager, "myScene");
	// Import the contents of the file into the scene.
	lImporter->Import(lScene);
	// The file is imported, so get rid of the importer.
	lImporter->Destroy();


	if (isTriangulated == false) {

		FbxGeometryConverter lGeomConverter(lSDKManager);
		lGeomConverter.Triangulate(lScene, true, false);
	}

	// Find mesh vertices and print information about them
	// Get the scenes root node
	FbxNode* lRootNode = lScene->GetRootNode();

	// Use root node to find children
	FbxNode* lNode = lRootNode->GetChild(0);
	int childCount = lRootNode->GetChildCount();

	if (lRootNode) {
		for (int i = 0; i < childCount; i++) {

			FbxNode* lNode = lRootNode->GetChild(i);
			FbxNodeAttribute * nodeAttribute = lNode->GetNodeAttribute();

			if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh)
			{
				if (CheckIfNodeHasSkeleton(lNode->GetParent()))
				{
					std::cout << "Animated mesh" << std::endl;
					animatedMesh(lNode, header, lScene, fileName);

					std::cout << "Anim name: " << header.animatedMesh[0].animations[0].animationName << std::endl; 
				}

				else
				{
					std::cout << "Static mesh" << std::endl;
					std::cout << std::endl;
					staticMesh(lNode, header, fileName);
				}
			}
		}

	}


	importr.SaveToFile(newName, header);

	std::cout << "File was saved" << std::endl;

	// Destroy the SDK manager and all the other objects it was handling.
	lSDKManager->Destroy();	struct VertexBlendingInfo {
		float blendingWeight;
		unsigned int blendingIndex;
	};


	getchar();
	return 0;
}
