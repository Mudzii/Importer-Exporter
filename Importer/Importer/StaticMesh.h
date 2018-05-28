#ifndef STATICMESH_H
#define STATICMESH_H

#include <stdio.h>
#include <vector>
#include <fstream>
#include <string>
#include "Mesh.h"

class StaticMesh : public Mesh {

public:
	std::string meshName;
	int staticMeshIndex;

	std::vector<StaticVertex> verticies;
	std::vector<unsigned int> indices;
	Material material;

public:
	StaticMesh();
	~StaticMesh();
	StaticMesh(std::string meshName, int staticMeshIndex, const std::vector<StaticVertex> &vertices,
		const std::vector<unsigned int> &indices, Material material);

	// =============

	int GetMeshIndex();
	int GetVertexCount();
	Material GetMaterial();
	std::string GetMeshName();
	unsigned int GetIndexCount();
	std::vector<unsigned int> GetIndices();
	std::vector<StaticVertex> GetVerticies();

	// =============

	void SetMeshIndex(int meshIndex);
	void SetMeshName(std::string meshName);
	void SetMeshMaterial(Material material);
	void SetIndices(std::vector<unsigned int> indices);
	void SetVertices(std::vector<StaticVertex> verticies);

	void PushBackIndice(unsigned int indice);
	void PushBackVertex(StaticVertex vertex);
	// =============


};

#endif