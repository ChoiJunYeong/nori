#include <nori/node.h>

NORI_NAMESPACE_BEGIN

Node::Node(BoundingBox3f box):
	m_bbox(box),
	m_leaf(false)
{ 
	for (int index = 0; index < OCTREE_NUM; index++) {
		Point3f x = m_bbox.getCenter();
		Point3f y = m_bbox.getCorner(index);
		for (int i = 0; i < 3; i++) {
			if (x[i] > y[i]) {
				std::swap(x[i], y[i]);
			}
		}
		m_child_box[index] = BoundingBox3f(x, y);
	}
}

Node::Node(BoundingBox3f box, std::vector<uint32_t> index_list):
	m_bbox(box),
	m_leaf(true),
	m_index_list(index_list)
{ }

Node::~Node(){
}

bool Node::overlap(const int index, const BoundingBox3f& bbox) const {
	try {
		return m_child_box[index].overlaps(bbox, true);
	}
	catch (std::exception& e) {
		std::cout << e.what();
	}
	return false;
}

void Node::addChild(std::unique_ptr<Node> node) {
	if (node != nullptr) {
		m_child_list.emplace_back(std::move(node));
	}
}

const BoundingBox3f& Node::GetChildBox(const int index) {
	return m_child_box[index];
}

void Node::getRayIntersectList(const Ray3f &ray, std::vector<uint32_t>& ret) const {
	if (!m_bbox.rayIntersect(ray))
		return;
	if (m_leaf) {
		ret.insert(ret.end(), m_index_list.begin(), m_index_list.end());
		return;
	}
	for (auto& child : m_child_list) {
		child->getRayIntersectList(ray,ret);
	}
}

NORI_NAMESPACE_END