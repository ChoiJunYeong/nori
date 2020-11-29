#include <nori/node.h>
#include <memory>

NORI_NAMESPACE_BEGIN

Node::Node(BoundingBox3f box):
	m_leaf(false)
{
	m_bbox = std::make_unique<BoundingBox3f>(box);
}
Node::Node(BoundingBox3f box, std::vector<uint32_t> index_list):
	m_leaf(true),
	m_index_list(index_list)
{ 
	m_bbox = std::make_unique<BoundingBox3f>(box);
}

Node::~Node(){
	std::cout << "asdasdsa";
}

bool Node::overlap(const int index, const BoundingBox3f& bbox) const {
	try {
		Point3f x = m_bbox->getCenter();
		Point3f y = m_bbox->getCorner(index);
		for (int i = 0; i < 3; i++) {
			if (x[i] > y[i]) {
				std::swap(x[i], y[i]);
			}
		}
		BoundingBox3f check_box(x, y);
		return check_box.overlaps(bbox, true);
	}
	catch (std::exception& e) {
		std::cout << e.what();
	}
	return false;
}

void Node::addChild(Node* node) {
	m_child_list.emplace_back(std::unique_ptr<Node>(node));
}

const BoundingBox3f Node::GetChildBox(const int index) const{
	Point3f x = m_bbox->getCenter();
	Point3f y = m_bbox->getCorner(index);
	for (int i = 0; i < 3; i++) {
		if (x[i] > y[i]) {
			std::swap(x[i], y[i]);
		}
	}
	BoundingBox3f check_box(x, y);
	return check_box;
}

uint32_t Node::rayIntersect(Ray3f &ray, Intersection &its, bool shadowRay, Mesh* mesh) {
	if (!m_bbox->rayIntersect(ray))
		return (uint32_t)-1;
	if (m_leaf) {
		uint32_t ret = (uint32_t)-1;
		for (auto idx : m_index_list) {
			float u, v, t;
			if (mesh->rayIntersect(idx, ray, u, v, t)) {
				/* An intersection was found! Can terminate
				   immediately if this is a shadow ray query */
				if (shadowRay)
					return (uint32_t)0;
				ray.maxt = its.t = t;
				its.uv = Point2f(u, v);
				its.mesh = mesh;
				ret = idx;
			}
		}
		return ret;
	}

	if (m_sort_origin != ray.o) {
		std::sort(m_child_list.begin(), m_child_list.end(),
			[&](const auto& a, const auto& b)
		{
			return a->m_bbox->distanceTo(ray.o) < b->m_bbox->distanceTo(ray.o);
		});
		m_sort_origin = ray.o;
	}
	
	for (auto& child : m_child_list) {
		uint32_t ret = child->rayIntersect(ray, its, shadowRay, mesh);
		if (ret != -1)
			return ret;
	}
	return (uint32_t)-1;
}


int Node::GetNodeMemorySize()  const {
	return sizeof(this);
}
int Node::GetTotalNodeNumber()  const {
	if (m_leaf)
		return 1;
	int ret = 1;
	for (auto& child : m_child_list) {
		ret += child->GetTotalNodeNumber();
	}
	return ret;
}
int Node::GetTotalLeafNumber()  const {
	if (m_leaf)
		return 1;
	int ret = 0;
	for (auto& child : m_child_list) {
		ret += child->GetTotalLeafNumber();
	}
	return ret;
}
int Node::GetTotalIndexNumberOfLeaf()  const {
	if (m_leaf)
		return m_index_list.size();
	int ret = 0;
	for (auto& child : m_child_list) {
		ret += child->GetTotalIndexNumberOfLeaf();
	}
	return ret;
}

NORI_NAMESPACE_END