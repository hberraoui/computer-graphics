#include <vector>

class PixelMap
{
  public:
    int width;
    int height;
    std::vector<uint32_t> pixels;

    PixelMap()
    {
      width = 0;
      height = 0;
      pixels = { };
    }

    PixelMap(int w, int h, std::vector<uint32_t> p)
    {
      width = w;
      height = h;
      pixels = p;
    }
};
