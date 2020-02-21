#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <PixelMap.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>

using namespace std;
using namespace glm;

#define WIDTH 430
#define HEIGHT 395
#define WHITE Colour(255,255,255)
#define RED Colour(255,0,0)
#define GREEN Colour(0,255,0)
#define BLUE Colour(0,0,255)
#define BLACK Colour(0,0,0)

void draw();
void update();
void handleEvent(SDL_Event event);
float interpolationStep(float start, float to, float from, int steps);
std::vector<float> interpolate(float from, float to, int steps);
std::vector<vec3> interpolate(vec3 from, vec3 to, int steps);
std::vector<vec3> interpolate(CanvasPoint from, CanvasPoint to, int steps);
int calcSteps(vec3 from, vec3 to);
int calcSteps(CanvasPoint from, CanvasPoint to);
float findXatYOnLine(float y, CanvasPoint from, CanvasPoint to);
bool drawPixelMap(PixelMap img, int startX, int startY);
bool drawPixelMap(PixelMap img);
PixelMap loadPixelMap(string fn);
void textureMappingTask();
void drawRandomFilledTriangle();
void drawFilledTriangle(CanvasTriangle t, PixelMap img);
void drawFilledTriangle(CanvasTriangle triangle, Colour colour);
void drawFilledTriangle(CanvasTriangle triangle);
void fillFlatBottomTriangle(CanvasTriangle t, Colour c);
void fillFlatTopTriangle(CanvasTriangle t, Colour c);
void drawRandomStrokedTriangle();
void drawStrokedTriangle(CanvasTriangle triangle, Colour colour);
void drawStrokedTriangle(CanvasTriangle triangle);
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
uint32_t vec3ToPackedInt(vec3 pixel);
vec3 packedIntToVec3(uint32_t colour);
void rainbowInterpolation();
void greyscaleInterpolation();

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);

int main(int argc, char* argv[])
{
  SDL_Event event;
  textureMappingTask();
  while(true)
  {
    // We MUST poll for events - otherwise the window will freeze !
    if(window.pollForInputEvents(&event)) handleEvent(event);
    update();
    draw();
    // Need to render the frame at the end, or nothing actually gets shown on the screen !
    window.renderFrame();
  }
}

// For the sake of Windows
int WinMain(int argc, char* argv[])
{
  main(argc, argv);
  return 0;
}

void draw()
{
  // window.clearPixels();
  // for(int y=0; y<window.height ;y++) {
  //   for(int x=0; x<window.width ;x++) {
  //     float red = rand() % 255;
  //     float green = 0.0;
  //     float blue = 0.0;
  //     uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
  //     window.setPixelColour(x, y, colour);
  //   }
  // }
}

void update()
{
  // Function for performing animation (shifting artifacts or moving the camera)
}

void handleEvent(SDL_Event event)
{
  if(event.type == SDL_KEYDOWN) {
    if(event.key.keysym.sym == SDLK_LEFT) cout << "LEFT" << endl;
    else if(event.key.keysym.sym == SDLK_RIGHT) cout << "RIGHT" << endl;
    else if(event.key.keysym.sym == SDLK_UP) cout << "UP" << endl;
    else if(event.key.keysym.sym == SDLK_DOWN) cout << "DOWN" << endl;
    else if(event.key.keysym.sym == SDLK_u) drawRandomStrokedTriangle();
    else if(event.key.keysym.sym == SDLK_f) drawRandomFilledTriangle();
  }
  else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "MOUSE CLICKED" << endl;
}

float interpolationStep(float start, float to, float from, int steps)
{
  return start + (to - from) / steps;
}

std::vector<float> interpolate(float from, float to, int steps)
{
  std::vector<float> v = { from };
    
  for (int i = 0; i < steps; ++i) {
    v.push_back(interpolationStep(v.back(),to,from,steps));
  }
  
  return v;
}

std::vector<vec3> interpolate(vec3 from, vec3 to, int steps)
{
  std::vector<vec3> v = { from };

  for (int i = 0; i < steps; ++i) {
    vec3 newvec(interpolationStep(v.back().x,to.x,from.x,steps),
                interpolationStep(v.back().y,to.y,from.y,steps),
                interpolationStep(v.back().z,to.z,from.z,steps));
    v.push_back(newvec);
  }

  return v;
}

std::vector<vec3> interpolate(CanvasPoint from, CanvasPoint to, int steps)
{
  vec3 fromVec(from.x,from.y,0);
  vec3 toVec(to.x,to.y,0);

  return interpolate(fromVec,toVec,steps);
}

int calcSteps(vec3 from, vec3 to)
{    
  float xDiff = to.x - from.x;
  float yDiff = to.y - from.y;
  return round(std::max(abs(xDiff), abs(yDiff)));
}

int calcSteps(CanvasPoint from, CanvasPoint to)
{    
  float xDiff = to.x - from.x;
  float yDiff = to.y - from.y;
  return round(std::max(abs(xDiff), abs(yDiff)));
}

float findXatYOnLine(float yTarget, CanvasPoint from, CanvasPoint to)
{
  float x = 0;
  
  int steps = calcSteps(from,to);  
  std::vector<vec3> points = interpolate(from, to, steps);
  for (int i=0; i<steps; i++) {
      if (round(points.at(i).y) == round(yTarget)) {
          x = points.at(i).x;
          break;
      }
  }  

  return x;
}

float findXatYOnLine(float yTarget, TexturePoint from, TexturePoint to)
{
  CanvasPoint fromNew(from.x,from.y);
  CanvasPoint toNew(to.x,to.y);
  return findXatYOnLine(yTarget,fromNew,toNew);
}

bool drawPixelMap(PixelMap img, int startX, int startY)
{
  if (img.pixels.size() > 0) {
    for (int y = startY; y < img.height; y++) {
      for (int x = startX; x < img.width; x++) {
        uint32_t pixel = img.pixels.at(x + img.width*y);
        if (x < window.width && y < window.height) {
          window.setPixelColour(x,y,pixel);
        }
      }      
    }
    return true;
  }
  return false;
}

bool drawPixelMap(PixelMap img)
{
  return drawPixelMap(img, 0, 0);
}

PixelMap loadPixelMap(string fn)
{
  PixelMap img;
  
  std::ifstream ifs;
  ifs.open(fn, ios::binary);
  if (ifs.is_open()) {
    string line;
    
    // check for "magic number" to indicate correct encoding method (P6)
    getline(ifs,line);
    if (line != "P6") {
      cout << "ERROR: Unsupported PPM format. Closing file." << endl;
      ifs.close();
      return img;
    }
    
    // ignore comments
    getline(ifs,line);
    while (line[0] == '#') getline(ifs,line);
    
    // read image width
    int width = std::stoi(line.substr(0, line.find(' ')));
    
    // read image height
    int height = std::stoi(line.substr(line.find(' '), line.length()));

    // read max colour value
    getline(ifs,line);
    int maxColourValue = std::stoi(line);
    
    // read the pixel data payload
    std::vector<uint32_t> texture = { };
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        char r, g, b;
        ifs.get(r);
        ifs.get(g);
        ifs.get(b);
        uint32_t pixel = (255<<24) + ((r % maxColourValue)<<16) + ((g % maxColourValue)<<8) + (b % maxColourValue);
        texture.push_back(pixel);
      }
    }

    ifs.close();
    
    img.width = width;
    img.height = height;
    img.pixels = texture;
  } else {
    cout << "ERROR: Cannot open file." << endl;
  }

  return img;
}

void textureMappingTask()
{
    TexturePoint t0(195,5);
    TexturePoint t1(395,380);
    TexturePoint t2(65,330);
    
    CanvasPoint v0(160,10,t0);
    CanvasPoint v1(300,230,t1);
    CanvasPoint v2(10,150,t2);
    
    CanvasTriangle t(v0,v1,v2,WHITE);
    
    PixelMap img = loadPixelMap("texture.ppm");
    drawFilledTriangle(t, img);
    
    drawStrokedTriangle(t);
}

void drawRandomFilledTriangle()
{
  CanvasPoint v0 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v1 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v2 = CanvasPoint(rand() % window.width, rand() % window.height);
  Colour colour = Colour(rand() % 255, rand() % 255, rand() % 255);
  CanvasTriangle triangle = CanvasTriangle(v0, v1, v2, colour);
  
  drawFilledTriangle(triangle);
}

void drawFilledTriangle(CanvasTriangle t, PixelMap img)
{
  // TODO: Properly implement Task 5: Texture Mapping
  // Sort vertices by vertical position (top to bottom)
  t.sortVertices();

  // Divide triangle into 2 "flat-bottomed" triangles
  float vy = t.vertices[1].y;
  float vx = findXatYOnLine(vy, t.vertices[0], t.vertices[2]);
  
  float vtpy = t.vertices[1].texturePoint.y;
  float vtpx = findXatYOnLine(vtpy, t.vertices[0].texturePoint, t.vertices[2].texturePoint);
  TexturePoint vtp = TexturePoint(vtpx,vtpy);
  CanvasPoint v(vx,vy,vtp);
  
  cout << v << "TexturePoint: " << v.texturePoint;
  
  CanvasTriangle topT = CanvasTriangle(t.vertices[0], t.vertices[1], v);
  CanvasTriangle bottomT = CanvasTriangle(t.vertices[1], v, t.vertices[2]);

  // Fill top triangle (top-to-bottom, left-to-right)
  for (int y = topT.vertices[0].y; y < topT.vertices[2].y; ++y) {
    float startX = findXatYOnLine(y, topT.vertices[0], topT.vertices[1]);
    float endX = findXatYOnLine(y, topT.vertices[0], topT.vertices[2]);
    CanvasPoint from(startX, y);
    CanvasPoint to(endX, y);
    int steps = calcSteps(from,to);
    std::vector<vec3> points = interpolate(from, to, steps);
    for (int i=0; i<steps; i++) {
      int x = round(points.at(i).x);
      int y = round(points.at(i).y);
      uint32_t pixel = img.pixels.at(x + img.width*y);
      window.setPixelColour(x, y, pixel);
    }
  }
  
  // Fill bottom triangle (top-to-bottom, left-to-right)
  for (int y = bottomT.vertices[0].y; y < bottomT.vertices[2].y; ++y) {
    float startX = findXatYOnLine(y, bottomT.vertices[0], bottomT.vertices[2]);
    float endX = findXatYOnLine(y, bottomT.vertices[1], bottomT.vertices[2]);
    CanvasPoint from(startX, y);
    CanvasPoint to(endX, y);
    int steps = calcSteps(from,to);
    std::vector<vec3> points = interpolate(from, to, steps);
    for (int i=0; i<steps; i++) {
      int x = round(points.at(i).x);
      int y = round(points.at(i).y);
      uint32_t pixel = img.pixels.at(x + img.width*y);
      window.setPixelColour(x, y, pixel);
    }
  }
}

void drawFilledTriangle(CanvasTriangle t, Colour colour)
{
  // Sort vertices by vertical position (top to bottom)
  t.sortVertices();
  
  // Divide triangle into 2 "flat-bottomed" triangles
  float vy = t.vertices[1].y;
  float vx = findXatYOnLine(vy, t.vertices[0], t.vertices[2]);
  CanvasPoint v(vx,vy);
  CanvasTriangle topT = CanvasTriangle(t.vertices[0], t.vertices[1], v);
  CanvasTriangle bottomT = CanvasTriangle(t.vertices[1], v, t.vertices[2]);

  // Fill top triangle (top-to-bottom, left-to-right)
  fillFlatBottomTriangle(topT, colour);
  
  // Fill bottom triangle (top-to-bottom, left-to-right)
  fillFlatTopTriangle(bottomT, colour);
  
  // Test the fill is accurate by drawing the original triangle
  drawStrokedTriangle(t, WHITE);
}

void drawFilledTriangle(CanvasTriangle t)
{
  drawFilledTriangle(t, t.colour);
}

void fillFlatBottomTriangle(CanvasTriangle t, Colour c)
{
  for (int y = t.vertices[0].y; y < t.vertices[2].y; ++y) {
    float startX = findXatYOnLine(y, t.vertices[0], t.vertices[1]);
    float endX = findXatYOnLine(y, t.vertices[0], t.vertices[2]);
    drawLine(CanvasPoint(startX, y), CanvasPoint(endX, y), c);
  }
}

void fillFlatTopTriangle(CanvasTriangle t, Colour c)
{
  // TODO: Merge with fillFlatBottomTriangle?
  for (int y = t.vertices[0].y; y < t.vertices[2].y; ++y) {
    float startX = findXatYOnLine(y, t.vertices[0], t.vertices[2]);
    float endX = findXatYOnLine(y, t.vertices[1], t.vertices[2]);
    drawLine(CanvasPoint(startX, y), CanvasPoint(endX, y), c);
  }
}

void drawRandomStrokedTriangle()
{
  CanvasPoint v0 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v1 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v2 = CanvasPoint(rand() % window.width, rand() % window.height);
  Colour colour = Colour(rand() % 255, rand() % 255, rand() % 255);
  CanvasTriangle triangle = CanvasTriangle(v0, v1, v2, colour);
  
  drawStrokedTriangle(triangle);
}

void drawStrokedTriangle(CanvasTriangle triangle, Colour colour)
{
  drawLine(triangle.vertices[0], triangle.vertices[1], colour);
  drawLine(triangle.vertices[0], triangle.vertices[2], colour);
  drawLine(triangle.vertices[1], triangle.vertices[2], colour);
}

void drawStrokedTriangle(CanvasTriangle triangle)
{
  drawStrokedTriangle(triangle, triangle.colour);
}

void drawLine(CanvasPoint from, CanvasPoint to, Colour colour)
{
  int steps = calcSteps(from,to);
  std::vector<vec3> points = interpolate(from, to, steps);
  for (int i=0; i<steps; i++) {
    window.setPixelColour(round(points.at(i).x), round(points.at(i).y), colour.toPackedInt());
  }
}

uint32_t vec3ToPackedInt(vec3 pixel)
{
  return (255<<24) + (int(pixel.x)<<16) + (int(pixel.y)<<8) + int(pixel.z);
}

vec3 packedIntToVec3(uint32_t colour)
{
  vec3 result( (colour<<8)>>24, (colour<<16)>>24, (colour<<24)>>24 );
  return result;
}

void rainbowInterpolation()
{
  // Draw a two dimension colour interpolation  
  window.clearPixels();
  vec3 redPixel(255,0,0);
  vec3 bluePixel(0,0,255);
  vec3 yellowPixel(255,255,0);
  vec3 greenPixel(0,255,0);
  window.setPixelColour(0, window.height-1, vec3ToPackedInt(yellowPixel));
  window.setPixelColour(window.width-1, window.height-1, vec3ToPackedInt(greenPixel));
  
  std::vector<vec3> v;
  v = interpolate(redPixel, yellowPixel, window.height);
  for(int y=0; y<window.height; y++) {
    window.setPixelColour(0, y, vec3ToPackedInt(v.at(y)));
  }
  
  v = interpolate(bluePixel, greenPixel, window.height);
  for(int y=0; y<window.height; y++) {
    window.setPixelColour(window.width-1, y, vec3ToPackedInt(v.at(y)));
  }

  for(int y=0; y<window.height; y++) {
    v = interpolate(packedIntToVec3(window.getPixelColour(0,y)),
                    packedIntToVec3(window.getPixelColour(window.width-1,y)),
                    window.width);
    for(int x=0; x<window.width; x++) {
      window.setPixelColour(x, y, vec3ToPackedInt(v.at(x)));
    }
  }
}

void greyscaleInterpolation()
{
  // Drawing a one dimension greyscale interpolation  
  window.clearPixels();
  std::vector<float> v;
  v = interpolate(0, 255, window.width);
  for(int y=0; y<window.height; y++) {
    for(int x=0; x<window.width; x++) {
      vec3 c(v.at(x), v.at(x), v.at(x));
      window.setPixelColour(x, y, vec3ToPackedInt(c));
    }
  }
}
