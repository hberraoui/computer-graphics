#include <glm/glm.hpp>
#include <iostream>

class RayTriangleIntersection
{
  public:
    glm::vec3 intersectionPoint;
    float distanceFromRayOrigin;
    ModelTriangle intersectedTriangle;

    RayTriangleIntersection()
    {
    }

    RayTriangleIntersection(glm::vec3 point, float distance, ModelTriangle triangle)
    {
        intersectionPoint = point;
        distanceFromRayOrigin = distance;
        intersectedTriangle = triangle;
    }
};

std::ostream& operator<<(std::ostream& os, const RayTriangleIntersection& intersection)
{
    os << "Intersection is at (" << intersection.intersectionPoint.x << "," << intersection.intersectionPoint.y << "," << intersection.intersectionPoint.z << ") on triangle:\n[TRIANGLE_START]\n" << intersection.intersectedTriangle << "[TRIANGLE_END]\nThe intersection took place at a distance of " << intersection.distanceFromRayOrigin << std::endl;
    return os;
}
