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
    Vector3D tMin = (min - r.o) / r.d;
    Vector3D tMax = (max - r.o) / r.d;


    // Optimization: loop unrolling
    for (int i = 0; i < 3; i++) {
        if (tMin[i] > tMax[i])
            std::swap(tMin[i], tMax[i]);
    }

    t0 = std::max({tMin.x, tMin.y, tMin.z});
    t1 = std::min({tMax.x, tMax.y, tMax.z});

    if (t0 > t1)
        return false;

    if (t0 > r.max_t || t1 < r.min_t)
        return false;

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
