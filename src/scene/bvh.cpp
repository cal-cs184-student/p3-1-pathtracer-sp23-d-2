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

  // If there are no more than max_leaf_size primitives in the list, the node we just created is a leaf node and we should update its start and end iterators appropriately.
  size_t size = distance(start, end);
  if (size <= max_leaf_size) {
    return node;
  }
  
  // split along axis
  Vector3D bboxExtent = bbox.extent;
  int splitAxis = 0;
  
  // Find longest axis for split
  if (bboxExtent.x > bboxExtent.y && bboxExtent.x > bboxExtent.z) {
      splitAxis = bboxExtent[0];
    } else if (bboxExtent.y > bboxExtent.x && bboxExtent.y > bboxExtent.z) {
      splitAxis = bboxExtent[1];
    } else {
      splitAxis = bboxExtent[2];
    }

  // find split point - midpoint of longest axis
  int splitPoint = floor((bbox.min[splitAxis] + bbox.max[splitAxis])/2);

  // split primitives into "left" and "right" collections based on split point
  auto partitionPoint = std::partition(start, end, [splitAxis, splitPoint](Primitive *p) {
    Vector3D bbCenter = p->get_bbox().centroid();
    return splitPoint > bbCenter[splitAxis];
  });
  
  // if the midpoint is either at the start or end, return current node as leaf node
  // prevents infinite loop
  if (start == partitionPoint || end == partitionPoint) {
    return node;
  }
  
  // update right and left of current node
  node->r = construct_bvh(partitionPoint, end, max_leaf_size);
  node->l = construct_bvh(start, partitionPoint, max_leaf_size);
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
    // TODO (Part 2.3):
    // Fill in the intersect function.
    // Take note that this function has a short-circuit that the
    // Intersection version cannot, since it returns as soon as it finds
    // a hit, it doesn't actually have to find the closest hit.



    for (auto p : primitives) {
        total_isects++;
        if (p->has_intersection(ray))
            return true;
    }
    return false;


}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
    // TODO (Part 2.3):
    // Fill in the intersect function.



    bool hit = false;
    for (auto p : primitives) {
        total_isects++;
        hit = p->intersect(ray, i) || hit;
    }
    return hit;


}

} // namespace SceneObjects
} // namespace CGL
