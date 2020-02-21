#include "CanvasPoint.h"
#include <iostream>

class CanvasTriangle
{
  public:
    CanvasPoint vertices[3];
    Colour colour;

    CanvasTriangle()
    {
    }

    CanvasTriangle(CanvasPoint v0, CanvasPoint v1, CanvasPoint v2)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = Colour(255,255,255);
    }

    CanvasTriangle(CanvasPoint v0, CanvasPoint v1, CanvasPoint v2, Colour c)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = c;
    }

    void sortVertices()
    {
      int n = sizeof(vertices) / sizeof(vertices[0]);
      bool swapped;
      for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
          swapped = false;
          if (vertices[j].y > vertices[j+1].y) {
            std::swap(vertices[j],vertices[j+1]);
            swapped = true;
          }
        }
        if (swapped == false)
          break;
      }
    }
};

std::ostream& operator<<(std::ostream& os, const CanvasTriangle& triangle)
{
    os << triangle.vertices[0]  << triangle.vertices[1]  << triangle.vertices[2] << std::endl;
    return os;
}
