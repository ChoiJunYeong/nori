/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/accel.h>
#include <Eigen/Geometry>
#include <chrono>
#include "tbb/tbb.h"
#include <mutex>
#include <memory>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

void Accel::build() {
	START_TIME_TRACK;
	std::vector<uint32_t> index_list;
	index_list.reserve(m_mesh->getTriangleCount());
	for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
		index_list.emplace_back(idx);
	}
	Node* node = build(m_bbox, index_list);
	m_root = std::unique_ptr<Node>(node);

	END_TIME_TRACK("build");

	std::cout << "memory size:" << sizeof(*m_root) << '\n';
	std::cout << "node number:" << m_root->GetTotalNodeNumber() << '\n';
	std::cout << "leaf number:" << m_root->GetTotalLeafNumber() << '\n';
	std::cout << "ave leaf node:" << m_root->GetTotalIndexNumberOfLeaf()/ m_root->GetTotalLeafNumber() << '\n';
}

Node* Accel::build(const BoundingBox3f& box, const std::vector<uint32_t>& index_list) const {
	if (index_list.size() == 0)
		return nullptr;
	if (index_list.size() <= 15 || m_bbox.getVolume()/ box.getVolume() > std::pow(OCTREE_NUM,10)) {
		return new Node(box,index_list);
	}
	Node* node = new Node(box);
	std::array<std::vector<uint32_t>, OCTREE_NUM> new_index_list;
	for (uint32_t index : index_list) {
		for (int i = 0; i < OCTREE_NUM; i++) {
			if (node->overlap(i,m_mesh->getBoundingBox(index))) {
				new_index_list[i].emplace_back(index);
			}
		}
	}
	std::mutex mu;
	tbb::parallel_for(0, OCTREE_NUM, [&](size_t i) {
		Node* child = build(node->GetChildBox(i), new_index_list[i]);
		mu.lock();
		if (child != nullptr) {
			node->addChild(child);
		}
		mu.unlock();
	});
	return node;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)
	uint32_t f = m_root->rayIntersect(ray, its, shadowRay, m_mesh);  // Triangle index of the closest intersection

    if (f != (uint32_t)-1) {
		if (shadowRay)
			return true;
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return f != (uint32_t)-1;
}

NORI_NAMESPACE_END

