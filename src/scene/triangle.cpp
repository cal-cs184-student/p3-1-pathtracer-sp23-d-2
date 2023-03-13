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
    Intersection _;
    return intersect(r, &_);
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
    // Part 1, Task 3:
    // implement ray-triangle intersection. When an intersection takes
    // place, the Intersection data should be updated accordingly
    Vector3D normal = cross(p3 - p1, p2 - p1);
    normal.normalize();
    double t = dot(p2 - r.o, normal) / dot(r.d, normal);
    Vector3D p = r.o + t * r.d;

    uint8_t count = 0;
    Vector3D vertices[] = {p1, p2, p3};
    for (int k = 0; k < 3; ++k) {
        Vector3D line = vertices[(k + 1) % 3] - vertices[k];
        Vector3D p_test_vector = p - vertices[k];
        if (cross(line, p_test_vector).norm() < 0)
            count++;
    }

    bool pointInTriangle = !count || count == 3;

    if (pointInTriangle && t >= r.min_t && t <= r.max_t) {
        isect->t = t;
        isect->n = normal;
        isect->primitive = this;
        isect->bsdf = get_bsdf();

        r.max_t = t;
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
