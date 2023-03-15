#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
    p1 = mesh->positions[v1];
    p2 = mesh->positions[v2];
    p3 = mesh->positions[v3];
    n1 = mesh->normals[v1];
    n2 = mesh->normals[v2];
    n3 = mesh->normals[v3];
    bbox = BBox(p1);
    bbox.expand(p2);
    bbox.expand(p3);

    bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
    // Part 1, Task 3: implement ray-triangle intersection
    // The difference between this function and the next function is that the next
    // function records the "intersection" while this function only tests whether
    // there is a intersection.

    // Möller Trumbore Algorithm
    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D s0 = r.o - p1;
    Vector3D s1 = cross(r.d, e2);
    Vector3D s2 = cross(s0, e1);

    Vector3D result = 1 / dot(s1, e1) * Vector3D(dot(s2, e2), dot(s1, s0), dot(s2, r.d));

    double t = result.x;
    Vector3D coord = Vector3D(1 - result.y - result.z, result.y, result.z);

    if (coord.x < 0 || coord.y < 0 || coord.z < 0) {
        return false;
    }

    if (coord.x > 1 || coord.y > 1 || coord.z > 1) {
        return false;
    }

    return t >= r.min_t && t <= r.max_t;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
    // Part 1, Task 3:
    // implement ray-triangle intersection. When an intersection takes
    // place, the Intersection data should be updated accordingly

    // Möller Trumbore Algorithm
    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D s0 = r.o - p1;
    Vector3D s1 = cross(r.d, e2);
    Vector3D s2 = cross(s0, e1);

    Vector3D result = 1 / dot(s1, e1) * Vector3D(dot(s2, e2), dot(s1, s0), dot(s2, r.d));

    Vector3D coord = Vector3D(1 - result.y - result.z, result.y, result.z);

    if (coord.x < 0 || coord.y < 0 || coord.z < 0) {
        return false;
    }

    if (coord.x > 1 || coord.y > 1 || coord.z > 1) {
        return false;
    }

    if (result.x >= r.min_t && result.x <= r.max_t) {
        isect->t = result.x;
        isect->n = coord.x * n1 + coord.y * n2 + coord.z * n3;
        isect->primitive = this;
        isect->bsdf = get_bsdf();
        r.max_t = result.x;

        return true;
    } else {
        return false;
    }
}

void Triangle::draw(const Color &c, float alpha) const {
    glColor4f(c.r, c.g, c.b, alpha);
    glBegin(GL_TRIANGLES);
    glVertex3d(p1.x, p1.y, p1.z);
    glVertex3d(p2.x, p2.y, p2.z);
    glVertex3d(p3.x, p3.y, p3.z);
    glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
    glColor4f(c.r, c.g, c.b, alpha);
    glBegin(GL_LINE_LOOP);
    glVertex3d(p1.x, p1.y, p1.z);
    glVertex3d(p2.x, p2.y, p2.z);
    glVertex3d(p3.x, p3.y, p3.z);
    glEnd();
}

} // namespace SceneObjects
} // namespace CGL
