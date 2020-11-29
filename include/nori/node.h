#pragma once

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN
#define OCTREE_NUM 8
class Node {
public:
	Node(BoundingBox3f box);
	Node(BoundingBox3f box, std::vector<uint32_t> index_list);
	Node(Node const &) = delete;
	void operator=(Node const &) = delete;
	~Node();
	bool overlap(const int index, const BoundingBox3f& bbox) const;
	uint32_t rayIntersect(Ray3f &ray, Intersection &its, bool shadowRay, Mesh* mesh);
	void addChild(Node* node);
	const BoundingBox3f GetChildBox(const int index) const;
public:
	std::unique_ptr<BoundingBox3f> m_bbox = nullptr;
private:
	std::vector<std::unique_ptr<Node>> m_child_list;
	std::vector<uint32_t> m_index_list;
	bool m_leaf;
	Point3f m_sort_origin{ 0 };
//profile node class 
public:
	int GetNodeMemorySize() const;
	int GetTotalNodeNumber() const;
	int GetTotalLeafNumber() const;
	int GetTotalIndexNumberOfLeaf() const;
};

NORI_NAMESPACE_END