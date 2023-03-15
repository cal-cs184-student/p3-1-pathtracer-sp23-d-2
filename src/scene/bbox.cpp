#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

    // TODO (Part 2.2):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // t0, t1, update t0 and t1 with the new intersection times.
    Vector3D tMin, tMax;

    // Optimization: loop unrolling
    tMin[0] = (min[0] - r.o[0]) / r.d[0];
    tMax[0] = (max[0] - r.o[0]) / r.d[0];
    if (tMin[0] > tMax[0])
        std::swap(tMin[0], tMax[0]);

    tMin[1] = (min[1] - r.o[1]) / r.d[1];
    tMax[1] = (max[1] - r.o[1]) / r.d[1];
    if (tMin[1] > tMax[1])
        std::swap(tMin[1], tMax[1]);

    tMin[2] = (min[2] - r.o[2]) / r.d[2];
    tMax[2] = (max[2] - r.o[2]) / r.d[2];
    if (tMin[2] > tMax[2])
        std::swap(tMin[2], tMax[2]);

    t0 = std::max({tMin.x, tMin.y, tMin.z});
    t1 = std::min({tMax.x, tMax.y, tMax.z});

    if (t0 > t1) {
        return false;
    }

    return true;

}

void BBox::draw(Color c, float alpha) const {

    glColor4f(c.r, c.g, c.b, alpha);

    // top
    glBegin(GL_LINE_STRIP);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(max.x, max.y, max.z);
    glEnd();

    // bottom
    glBegin(GL_LINE_STRIP);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, min.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glEnd();

    // side
    glBegin(GL_LINES);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(min.x, min.y, max.z);
    glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
    return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
