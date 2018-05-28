#include "StaticMesh.h"

StaticMesh::StaticMesh(std::string meshName, int staticMeshIndex, const std::vector<StaticVertex> &vertices,
	const std::vector<unsigned int> &indices, Material material) {

	this->meshName = meshName;
	this->staticMeshIndex = staticMeshIndex;

	this->indices = indices;
	this->verticies = vertices;
	this->material = material;

}

StaticMesh::StaticMesh()
{
	this->meshName = "";
	this->staticMeshIndex = 0;

}

StaticMesh::~StaticMesh() {

}

// =============

int StaticMesh::GetMeshIndex() {
	return this->staticMeshIndex;
}

int StaticMesh::GetVertexCount() {
	return this->verticies.size();
}

std::string StaticMesh::GetMeshName() {
	return this->meshName;
}

Mesh::Material StaticMesh::GetMaterial() {
	return this->material;
}

unsigned int StaticMesh::GetIndexCount() {
	return this->indices.size();
}


std::vector<unsigned int> StaticMesh::GetIndices() {
	return this->indices;
}

std::vector<Mesh::StaticVertex> StaticMesh::GetVerticies() {
	return this->verticies;
}

// =============

void StaticMesh::SetMeshIndex(int meshIndex) {
	this->staticMeshIndex = meshIndex;
}

void StaticMesh::SetMeshName(std::string meshName) {
	this->meshName = meshName;
}

void StaticMesh::SetMeshMaterial(Material material) {
	this->material = material;
}

void StaticMesh::SetIndices(std::vector<unsigned int> indices) {
	this->indices = indices;
}

void StaticMesh::SetVertices(std::vector<StaticVertex> verticies) {
	this->verticies = verticies;
}


void StaticMesh::PushBackIndice(unsigned int indice) {
	this->indices.push_back(indice);
}

void StaticMesh::PushBackVertex(StaticVertex vertex) {
	this->verticies.push_back(vertex);
}
