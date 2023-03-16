#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

    primitives = std::vector<Primitive *>(_primitives);
    root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
    if (root)
        delete root;
    primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
    if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
            (*p)->draw(c, alpha);
        }
    } else {
        draw(node->l, c, alpha);
        draw(node->r, c, alpha);
    }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
    if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
            (*p)->drawOutline(c, alpha);
        }
    } else {
        drawOutline(node->l, c, alpha);
        drawOutline(node->r, c, alpha);
    }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

    // TODO (Part 2.1):
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code build a BVH aggregate with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    BBox bbox;
    for (auto p = start; p != end; p++) {
        BBox bb = (*p)->get_bbox();
        bbox.expand(bb);
    }

    BVHNode *node = new BVHNode(bbox);
    node->start = start;
    node->end = end;
    node->l = NULL;
    node->r = NULL;

    // If there are no more than max_leaf_size primitives in the list, the node we just created is a leaf node and we should update its start and end iterators appropriately.
    size_t size = distance(start, end);
    if (size <= max_leaf_size)
        return node;

    double totalArea = accumulate(start, end, 0., [](double acc, const Primitive *rhs) {
        return acc + rhs->get_bbox().surface_area();
    });

    vector<double> extent = {bbox.extent.x ,bbox.extent.y, bbox.extent.y};
    int splitAxis = (int) (min_element(extent.begin(), extent.end()) - extent.begin());
    vector<Primitive *> sortedAxis;
    copy(start, end, back_inserter(sortedAxis));
    sort(sortedAxis.begin(), sortedAxis.end(), [splitAxis](Primitive *lhs, Primitive *rhs){
        double lhsAxisValue = lhs->get_bbox().centroid()[splitAxis];
        double rhsAxisValue = rhs->get_bbox().centroid()[splitAxis];
        if (lhsAxisValue == rhsAxisValue) {
            return lhs->get_bbox().surface_area() < rhs->get_bbox().surface_area();
        } else {
            return lhs->get_bbox().centroid()[splitAxis] < rhs->get_bbox().centroid()[splitAxis];
        }
    });
    double splitPoint = sortedAxis[size / 2]->get_bbox().centroid()[splitAxis];

//    vector<double> splitAreas = {0., 0., 0.};
//    double axisSplitPoints[] {0., 0., 0.};
//
//    // Assuming the probability of intersection is proportional to the
//    // surface area of the bounding box, we want the split to result in
//    // maximum entropy reduction. Since we are splitting two ways, that
//    // means the probability on each side should be as close to 0.5 as
//    // possible.
//    for (int i = 0; i < 3; i++) {
//        vector<Primitive *> axis;
//        copy(start, end, back_inserter(axis));
//        sort(axis.begin(), axis.end(), [i](Primitive *lhs, Primitive *rhs){
//            double lhsAxisValue = lhs->get_bbox().centroid()[i];
//            double rhsAxisValue = rhs->get_bbox().centroid()[i];
//            if (lhsAxisValue == rhsAxisValue) {
//                return lhs->get_bbox().surface_area() < rhs->get_bbox().surface_area();
//            } else {
//                return lhs->get_bbox().centroid()[i] < rhs->get_bbox().centroid()[i];
//            }
//        });
//
//        splitAreas[i] = -totalArea / 2;
//
//        for (auto p = axis.begin(); p != axis.end(); p++) {
//            splitAreas[i] += (*p)->get_bbox().surface_area();
//            axisSplitPoints[i] = (*p)->get_bbox().centroid()[i];
//            if (splitAreas[i] > 0) {
//                break;
//            }
//        }
//    }
//
//
//    int splitAxis = (int) (min_element(splitAreas.begin(), splitAreas.end()) - splitAreas.begin());
//    double splitPoint = axisSplitPoints[splitAxis];

    // Find longest axis for split
//    Vector3D bboxExtent = bbox.extent;
//    if (bboxExtent.x > bboxExtent.y && bboxExtent.x > bboxExtent.z) {
//        splitAxis = 0;
//    } else if (bboxExtent.y > bboxExtent.x && bboxExtent.y > bboxExtent.z) {
//        splitAxis = 1;
//    } else {
//        splitAxis = 2;
//    }
//    splitPoint = (bbox.min[splitAxis] + bbox.max[splitAxis]) / 2;

    auto partitionPoint = partition(start, end, [splitAxis, splitPoint](Primitive *p) {
        Vector3D bbCenter = p->get_bbox().centroid();
        return bbCenter[splitAxis] <= splitPoint;
    });

    if (start == partitionPoint || end == partitionPoint)
        return node;

    node->r = construct_bvh(partitionPoint, end, max_leaf_size);
    node->l = construct_bvh(start, partitionPoint, max_leaf_size);

    node->start = node->l->start;
    node->end = node->l->end;
    return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
    // TODO (Part 2.3):
    // Fill in the intersect function.
    // Take note that this function has a short-circuit that the
    // Intersection version cannot, since it returns as soon as it finds
    // a hit, it doesn't actually have to find the closest hit.

    double t0, t1;
    if (!node->bb.intersect(ray, t0, t1))
        return false;

    if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
            total_isects++;
            if ((*p)->has_intersection(ray))
                return true;
        }

        return false;
    }

    // Here we are safe t short circuit
    return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
    // TODO (Part 2.3):
    // Fill in the intersect function.

    double t0, t1;
    if (!node->bb.intersect(ray, t0, t1))
        return false;

    if (node->isLeaf()) {
        bool hit = false;
        for (auto p = node->start; p != node->end; p++) {
            total_isects++;
            hit = (*p)->intersect(ray, i) || hit;
        }

        return hit;
    }

    // Use bitwise op to prevent short-circuiting
    return intersect(ray, i, node->l) | intersect(ray, i, node->r);
}

} // namespace SceneObjects
} // namespace CGL
