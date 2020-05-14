#include <glm/glm.hpp>
#include "Colour.h"
#include <CanvasTriangle.h>
#include <string>

class ModelTriangle
{
  public:
    glm::vec3 vertices[3];
    TexturePoint texturePoints[3];
    Colour colour;

    ModelTriangle()
    {
      texturePoints[0] = TexturePoint(-1,-1);
      texturePoints[1] = TexturePoint(-1,-1);
      texturePoints[2] = TexturePoint(-1,-1);
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, Colour trigColour, TexturePoint texPt0, TexturePoint texPt1, TexturePoint texPt2)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = trigColour;
      texturePoints[0] = texPt0;
      texturePoints[1] = texPt1;
      texturePoints[2] = texPt2;
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, Colour trigColour)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = trigColour;
      texturePoints[0] = TexturePoint(-1,-1);
      texturePoints[1] = TexturePoint(-1,-1);
      texturePoints[2] = TexturePoint(-1,-1);
    }
};

std::ostream& operator<<(std::ostream& os, const ModelTriangle& triangle)
{
    os << "(" << triangle.vertices[0].x << ", " << triangle.vertices[0].y << ", " << triangle.vertices[0].z << ")" << std::endl;
    os << "(" << triangle.vertices[1].x << ", " << triangle.vertices[1].y << ", " << triangle.vertices[1].z << ")" << std::endl;
    os << "(" << triangle.vertices[2].x << ", " << triangle.vertices[2].y << ", " << triangle.vertices[2].z << ")" << std::endl;
    os << std::endl;
    return os;
}
