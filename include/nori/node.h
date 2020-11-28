#pragma once

#include <nori/object.h>
#include <nori/bbox.h>
#include <nori/mesh.h>
#include <array>

NORI_NAMESPACE_BEGIN
#define OCTREE_NUM 8
class Node {
public:
	Node(BoundingBox3f box);
	Node(BoundingBox3f box, std::vector<uint32_t> index_list);
	~Node();
	bool overlap(const int index, const BoundingBox3f& bbox) const;
	void getRayIntersectList(const Ray3f &ray, std::vector<uint32_t>& ret) const;
	void addChild(std::unique_ptr<Node> node);
	const BoundingBox3f& GetChildBox(const int index);
private:
	BoundingBox3f m_bbox;
	std::vector<std::unique_ptr<Node>> m_child_list;
	std::vector<uint32_t> m_index_list;
	std::array<BoundingBox3f, OCTREE_NUM> m_child_box;
	bool m_leaf;
};

NORI_NAMESPACE_END