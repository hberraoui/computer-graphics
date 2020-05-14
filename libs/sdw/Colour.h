#include <iostream>

#define WHITE Colour(255,255,255)
#define RED Colour(255,0,0)
#define GREEN Colour(0,255,0)
#define BLUE Colour(0,0,255)
#define BLACK Colour(0,0,0)

class Colour
{
  public:
    std::string name;
    int red;
    int green;
    int blue;

    Colour()
    {
    }

    Colour(int r, int g, int b)
    {
      name = "";
      red = r;
      green = g;
      blue = b;
    }

    Colour(std::string n, int r, int g, int b)
    {
      name = n;
      red = r;
      green = g;
      blue = b;
    }
    
    uint32_t toPackedInt()
    {
      return (255<<24) + (this->red<<16) + (this->green<<8) + this->blue;
    }
};

std::ostream& operator<<(std::ostream& os, const Colour& colour)
{
    os << colour.name << " [" << colour.red << ", " << colour.green << ", " << colour.blue << "]" << std::endl;
    return os;
}