#ifndef MESH_H
#define MESH_H

class Mesh {

public: 
	struct Matrix4x4
	{
		float column1[4];
		float column2[4];
		float column3[4];
		float column4[4];
	};

	struct StaticVertex {
		float position[3];
		float normal[3];
		float UV[2];
		float tangent[3];
		float bitangent[3];
	};

	struct AnimatedVertex {
		float position[3];
		float normal[3];
		float UV[2];
		float tangent[3];
		float bitangent[3];
		int boneIndicies[4]; 
		float boneWeight[4];
	};


	struct Joint {
		char boneName[50]; 
		int parentIndex;
		unsigned int jointIndex;
		/*float rotation[4];
		float transform[4];
		float localTransformMat[4][4];*/
	};

	struct Animation {
		char animationName[50];
		unsigned int animationIndex;
		unsigned int nrOfFrames; 
	};

	struct Material {
		//float diffuse[4];
		//float specular[4];
		char diffuseTexture[50];
		char normalTexture[50];
		//char specularTexture[50];
	};

};




#endif